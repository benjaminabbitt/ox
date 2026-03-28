//! Ox Robotics Microkernel
//!
//! Features:
//! - 1kHz control loop (<1ms latency)
//! - Motor and sensor server integration
//! - WiFi telemetry (feature: wifi) - TODO: version compatibility
//! - BLE controller input (feature: ble) - TODO
//! - RC receiver support (feature: rc) - SBUS, CRSF/ELRS
//!
//! Wiring:
//! - GPIO 2: Green LED (status)
//! - GPIO 3: Red LED (fault indicator)
//! - GPIO 9: Button (mode switch)
//! - GPIO 4: I2C SDA (MPU6050)
//! - GPIO 5: I2C SCL (MPU6050)
//! - GPIO 6: Servo PWM
//! - GPIO 20: CRSF RX (when rc feature enabled)
//! - GPIO 21: CRSF TX (when rc feature enabled)

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::timer::timg::TimerGroup;

// Services
use ox_services::motor::{MotorCommand, MotorServer, MotorServerConfig};
use ox_services::sensor::{ImuRaw, SensorServer};

// HAL mocks for simulation
use ox_hal::mock::{MockEncoder, MockMotor};

// RC imports
#[cfg(feature = "rc")]
use {
    esp_hal::uart::{Config as UartConfig, Uart, UartRx},
    ox_services::rc::{CrsfDecoder, RcServer, ChannelMap, FailsafeConfig},
};

// Control loop timing target
const CONTROL_PERIOD_US: u64 = 1000; // 1ms = 1kHz

// IPC channels
static MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

#[cfg(feature = "rc")]
static RC_MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

// Timing statistics (thread-safe via Mutex)
static TIMING_STATS: Mutex<CriticalSectionRawMutex, TimingStats> =
    Mutex::new(TimingStats::new());

/// Timing statistics for control loop
#[derive(Clone, Copy)]
struct TimingStats {
    min_us: u64,
    max_us: u64,
    total_us: u64,
    count: u32,
}

impl TimingStats {
    const fn new() -> Self {
        Self {
            min_us: u64::MAX,
            max_us: 0,
            total_us: 0,
            count: 0,
        }
    }

    fn record(&mut self, duration_us: u64) {
        self.min_us = self.min_us.min(duration_us);
        self.max_us = self.max_us.max(duration_us);
        self.total_us += duration_us;
        self.count += 1;
    }

    fn avg_us(&self) -> u64 {
        if self.count > 0 {
            self.total_us / self.count as u64
        } else {
            0
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize ESP-HAL
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize Embassy time driver
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize logging
    esp_println::logger::init_logger_from_env();

    log::info!("================================");
    log::info!("    Ox Robotics Microkernel");
    log::info!("================================");
    log::info!("Chip: {}", ox_chip::CHIP.name);
    log::info!("Target control rate: {} Hz", 1_000_000 / CONTROL_PERIOD_US);

    // Feature flags
    #[cfg(feature = "wifi")]
    log::info!("WiFi: ENABLED (not yet implemented)");
    #[cfg(not(feature = "wifi"))]
    log::info!("WiFi: disabled");

    #[cfg(feature = "ble")]
    log::info!("BLE: ENABLED (not yet implemented)");
    #[cfg(not(feature = "ble"))]
    log::info!("BLE: disabled");

    #[cfg(feature = "rc")]
    log::info!("RC: ENABLED (CRSF on GPIO20/21)");
    #[cfg(not(feature = "rc"))]
    log::info!("RC: disabled");

    // ========== GPIO Setup ==========
    let led_green = Output::new(peripherals.GPIO2, Level::Low);
    let led_red = Output::new(peripherals.GPIO3, Level::Low);
    let button = Input::new(peripherals.GPIO9, Pull::Up);

    log::info!("[GPIO] Status LED on GPIO2, Fault LED on GPIO3");
    log::info!("[GPIO] Mode button on GPIO9");

    // ========== CRSF/RC Setup ==========
    #[cfg(feature = "rc")]
    {
        log::info!("[RC] Initializing CRSF receiver on UART...");

        // CRSF uses 420000 baud
        let uart_config = UartConfig::default().with_baudrate(420_000);

        let uart = Uart::new(peripherals.UART1, uart_config)
            .unwrap()
            .with_rx(peripherals.GPIO20)
            .with_tx(peripherals.GPIO21)
            .into_async();

        // We only need the RX part for receiving CRSF
        let (rx, _tx) = uart.split();

        spawner.spawn(crsf_receiver_task(rx)).unwrap();
        log::info!("[RC] CRSF receiver task spawned");
    }

    // ========== Core Tasks ==========
    spawner.spawn(control_loop_task()).unwrap();
    spawner.spawn(status_led_task(led_green)).unwrap();
    spawner.spawn(fault_led_task(led_red)).unwrap();
    spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(timing_report_task()).unwrap();

    log::info!("All tasks spawned");
    log::info!("================================");
    log::info!("Press button to cycle control modes:");
    log::info!("  Coast -> Velocity -> Position -> E-Stop");
    log::info!("================================");

    // Main supervisor loop
    loop {
        Timer::after(Duration::from_secs(10)).await;
        // Could add watchdog/health checks here
    }
}

/// Control loop task - runs motor and sensor servers at 1kHz
#[embassy_executor::task]
async fn control_loop_task() {
    log::info!("[CTRL] Starting 1kHz control loop");

    // Create motor server with mock hardware
    let mut motor_server = MotorServer::new(
        MockMotor::new(),
        MockMotor::new(),
        MockEncoder::new(1000),
        MockEncoder::new(1000),
        MotorServerConfig::default(),
    );

    // Create sensor server
    let mut sensor_server = SensorServer::default();

    // Simulated IMU data (as if robot is level)
    let imu_raw = ImuRaw {
        accel_z: 16384, // 1g on Z axis
        ..Default::default()
    };

    let dt = CONTROL_PERIOD_US as f32 / 1_000_000.0;
    let mut iteration = 0u32;

    loop {
        let start = Instant::now();

        // 1. Process button commands
        if let Ok(cmd) = MOTOR_CMD.try_receive() {
            log::info!("[CTRL] Button command: {:?}", cmd);
            motor_server.process_command(cmd);
        }

        // 2. Process RC commands (higher priority)
        #[cfg(feature = "rc")]
        if let Ok(cmd) = RC_MOTOR_CMD.try_receive() {
            motor_server.process_command(cmd);
        }

        // 3. Process sensor data
        let _imu = sensor_server.process_imu(imu_raw, dt);

        // 4. Run motor control loop
        motor_server.update(dt);

        // 5. Record timing
        let elapsed_us = start.elapsed().as_micros() as u64;
        {
            let mut stats = TIMING_STATS.lock().await;
            stats.record(elapsed_us);
        }

        // Log occasional status
        iteration += 1;
        if iteration % 1000 == 0 {
            let status = motor_server.status();
            log::info!(
                "[CTRL] Mode: {:?}, Iter: {}, Loop: {}us",
                status.mode,
                iteration,
                elapsed_us
            );
        }

        // Wait for next control period
        let remaining = CONTROL_PERIOD_US.saturating_sub(elapsed_us);
        if remaining > 0 {
            Timer::after(Duration::from_micros(remaining)).await;
        }
    }
}

// ========== CRSF/RC Task ==========

#[cfg(feature = "rc")]
#[embassy_executor::task]
async fn crsf_receiver_task(mut rx: UartRx<'static, esp_hal::Async>) {
    use embedded_io_async::Read;

    log::info!("[CRSF] Starting CRSF receiver task");

    let mut decoder = CrsfDecoder::new();
    let mut rc_server = RcServer::new(ChannelMap::default(), FailsafeConfig::default());

    let mut buf = [0u8; 1];
    let mut last_update = Instant::now();

    loop {
        // Read one byte at a time (CRSF is byte-oriented)
        match rx.read(&mut buf).await {
            Ok(_) => {
                if let Some(channels) = decoder.decode(buf[0]) {
                    rc_server.process(&channels);

                    // Convert RC to motor command
                    let throttle = rc_server.throttle();
                    let steering = rc_server.steering();

                    // Skip if in failsafe
                    if !rc_server.is_failsafe() {
                        // Differential mixing
                        let left = ((throttle + steering) * 1000.0) as i16;
                        let right = ((throttle - steering) * 1000.0) as i16;

                        let cmd = if rc_server.is_armed() {
                            MotorCommand::SetVelocity { left, right }
                        } else {
                            MotorCommand::Coast
                        };

                        let _ = RC_MOTOR_CMD.try_send(cmd);
                    }

                    // Log occasionally
                    if decoder.link_quality() > 0 {
                        log::debug!(
                            "[CRSF] T:{:.2} S:{:.2} LQ:{} RSSI:{}",
                            throttle,
                            steering,
                            decoder.link_quality(),
                            decoder.rssi()
                        );
                    }
                }
            }
            Err(e) => {
                log::warn!("[CRSF] UART error: {:?}", e);
                Timer::after(Duration::from_millis(10)).await;
            }
        }

        // Update failsafe timer
        let now = Instant::now();
        let elapsed = now.duration_since(last_update).as_millis() as u32;
        rc_server.update(elapsed);
        last_update = now;
    }
}

// ========== LED and Button Tasks ==========

/// Status LED task - indicates control mode
#[embassy_executor::task]
async fn status_led_task(mut led: Output<'static>) {
    log::info!("[LED] Status LED task started");

    loop {
        // Blink pattern indicates system is running
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(900)).await;
    }
}

/// Fault LED task - would indicate errors
#[embassy_executor::task]
async fn fault_led_task(mut led: Output<'static>) {
    // Keep fault LED off (no faults in demo)
    led.set_low();

    // In a real system, this would monitor for faults
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

/// Button task - cycles through control modes
#[embassy_executor::task]
async fn button_task(button: Input<'static>) {
    log::info!("[BTN] Button task started");

    let mut last_state = true;
    let mut mode_index = 0u8;

    let modes = [
        MotorCommand::Coast,
        MotorCommand::SetVelocity { left: 500, right: 500 },
        MotorCommand::SetPosition { left: 1000, right: 1000 },
        MotorCommand::EmergencyStop,
    ];

    loop {
        let current = button.is_high();

        // Detect falling edge
        if last_state && !current {
            mode_index = (mode_index + 1) % modes.len() as u8;
            let cmd = modes[mode_index as usize];

            log::info!("[BTN] Button pressed - sending {:?}", cmd);
            let _ = MOTOR_CMD.try_send(cmd);
        }
        last_state = current;

        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Timing report task - prints control loop statistics
#[embassy_executor::task]
async fn timing_report_task() {
    log::info!("[TIME] Timing report task started");

    // Wait for control loop to stabilize
    Timer::after(Duration::from_secs(2)).await;

    loop {
        Timer::after(Duration::from_secs(5)).await;

        let mut stats = TIMING_STATS.lock().await;

        if stats.count > 0 {
            let meets_target = stats.max_us < CONTROL_PERIOD_US;
            let status = if meets_target { "OK" } else { "EXCEEDED" };

            log::info!("================================");
            log::info!("[TIME] Control Loop Statistics:");
            log::info!("  Iterations: {}", stats.count);
            log::info!("  Min: {}us", stats.min_us);
            log::info!("  Max: {}us", stats.max_us);
            log::info!("  Avg: {}us", stats.avg_us());
            log::info!("  Target: {}us - {}", CONTROL_PERIOD_US, status);
            log::info!("================================");

            // Reset stats for next period
            stats.reset();
        }
    }
}
