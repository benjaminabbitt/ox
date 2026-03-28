//! Ox Robotics Microkernel - MVP Demo
//!
//! Demonstrates the microkernel architecture with:
//! - Control loop timing verification (<1ms target)
//! - GPIO server integration
//! - Sensor fusion (simulated IMU data)
//! - Motor control (simulated)
//!
//! Wiring (diagram.json):
//! - GPIO 2: Green LED (status)
//! - GPIO 3: Red LED (fault indicator)
//! - GPIO 9: Button (mode switch)
//! - GPIO 4: I2C SDA (MPU6050)
//! - GPIO 5: I2C SCL (MPU6050)
//! - GPIO 6: Servo PWM

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

// Control loop timing target
const CONTROL_PERIOD_US: u64 = 1000; // 1ms = 1kHz

// IPC channels
static MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

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
    log::info!("    Ox Microkernel MVP Demo");
    log::info!("================================");
    log::info!("Chip: {}", ox_chip::CHIP.name);
    log::info!("Target control rate: {} Hz", 1_000_000 / CONTROL_PERIOD_US);

    // ========== GPIO Setup ==========
    let led_green = Output::new(peripherals.GPIO2, Level::Low);
    let led_red = Output::new(peripherals.GPIO3, Level::Low);
    let button = Input::new(peripherals.GPIO9, Pull::Up);

    log::info!("[GPIO] Status LED on GPIO2, Fault LED on GPIO3");
    log::info!("[GPIO] Mode button on GPIO9");

    // ========== Spawn Tasks ==========
    spawner.spawn(control_loop_task()).unwrap();
    spawner.spawn(status_led_task(led_green)).unwrap();
    spawner.spawn(fault_led_task(led_red)).unwrap();
    spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(timing_report_task()).unwrap();

    log::info!("All tasks spawned");
    log::info!("================================");
    log::info!("Press button to cycle control modes:");
    log::info!("  Coast -> Velocity -> Position -> Coast");
    log::info!("================================");

    // Main supervisor loop
    loop {
        Timer::after(Duration::from_secs(10)).await;
        // Could add watchdog/health checks here
    }
}

/// Control loop task - runs motor and sensor servers at 1kHz
///
/// This is the heart of the microkernel - demonstrating:
/// - Precise timing control
/// - Server integration (motor, sensor)
/// - IPC command processing
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

        // 1. Process any pending commands
        if let Ok(cmd) = MOTOR_CMD.try_receive() {
            log::info!("[CTRL] Command: {:?}", cmd);
            motor_server.process_command(cmd);
        }

        // 2. Process sensor data
        let _imu = sensor_server.process_imu(imu_raw, dt);

        // 3. Run motor control loop
        motor_server.update(dt);

        // 4. Record timing
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
