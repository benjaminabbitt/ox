//! Ox Robotics Microkernel
//!
//! Features:
//! - 1kHz control loop (<1ms latency)
//! - Motor and sensor server integration
//! - WiFi telemetry (feature: wifi)
//! - BLE controller input (feature: ble) - HCI layer initialized
//! - RC receiver support (feature: rc) - SBUS, CRSF/ELRS
//! - GPS receiver (feature: gps) - NMEA/UBX protocols
//! - Compass/magnetometer (feature: compass) - QMC5883L/HMC5883L
//! - Autonomous navigation (feature: nav) - waypoint following
//!
//! Wiring:
//! - GPIO 2: Green LED (status)
//! - GPIO 3: Red LED (fault indicator)
//! - GPIO 9: Button (mode switch)
//! - GPIO 4: I2C SDA (MPU6050, Compass)
//! - GPIO 5: I2C SCL (MPU6050, Compass)
//! - GPIO 6: Servo PWM
//! - GPIO 16: GPS TX -> ESP RX (when gps feature enabled)
//! - GPIO 17: GPS RX <- ESP TX (when gps feature enabled)
//! - GPIO 20: CRSF RX (when rc feature enabled)
//! - GPIO 21: CRSF TX (when rc feature enabled)

#![no_std]
#![no_main]

mod logging;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_println as _; // Keep defmt global logger

// Provide defmt timestamp
defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::timer::timg::TimerGroup;

// Rate extension for I2C frequency (compass)
#[cfg(feature = "compass")]
use fugit::RateExtU32;

use logging::*;

// Services
use ox_services::motor::{MotorCommand, MotorServer, MotorServerConfig};
use ox_services::sensor::{ImuRaw, SensorServer};

// HAL mocks for simulation
use ox_hal::mock::{MockEncoder, MockMotor};

// UART imports (shared by RC and GPS)
#[cfg(any(feature = "rc", feature = "gps"))]
use esp_hal::uart::{Config as UartConfig, Uart, UartRx};

#[cfg(feature = "gps")]
use esp_hal::uart::UartTx;

// RC imports
#[cfg(feature = "rc")]
use ox_services::rc::{CrsfDecoder, RcServer, ChannelMap, FailsafeConfig};

// GPS imports
#[cfg(feature = "gps")]
use ox_services::gps::{GpsConfig, GpsData, GpsServer, UbxConfigBuilder};

// Compass imports
#[cfg(feature = "compass")]
use {
    esp_hal::i2c::master::I2c,
    ox_services::compass::{CompassConfig, CompassData, CompassServer, Qmc5883l},
};

// Navigation imports
#[cfg(feature = "nav")]
use ox_services::nav::{NavCommand, NavConfig, NavServer, NavTelemetry};

// Vehicle state machine imports
#[cfg(feature = "vehicle")]
use ox_services::vehicle::{VehicleCommand, VehicleMode, VehicleStateMachine, TransitionResult};

// WiFi imports
#[cfg(feature = "wifi")]
use {
    embassy_net::{Config as NetConfig, Runner, Stack, StackResources},
    esp_wifi::wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice,
        WifiEvent as EspWifiEvent, WifiStaDevice,
    },
    ox_services::comms::{CommsConfig, CommsServer},
};
// Use local logging types (not esp_wifi's)
#[cfg(feature = "wifi")]
use crate::logging::{WifiEvent, WifiState};
// Nav telemetry needs these when both nav and wifi enabled
#[cfg(all(feature = "nav", feature = "wifi"))]
use ox_services::comms::{TelemetryMessage, TelemetryType};

// BLE imports
#[cfg(feature = "ble")]
use esp_wifi::ble::controller::BleConnector;

// WiFi/BLE shared imports
#[cfg(any(feature = "wifi", feature = "ble"))]
use {
    esp_hal::rng::Rng,
    esp_wifi::{init as wifi_init, EspWifiController},
};

// WiFi configuration (set via environment or use defaults)
#[cfg(feature = "wifi")]
const WIFI_SSID: &str = env!("WIFI_SSID");
#[cfg(feature = "wifi")]
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

// Helper macro to create static variables at runtime (used by wifi and ble)
#[cfg(any(feature = "wifi", feature = "ble"))]
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

// Control loop timing target
const CONTROL_PERIOD_US: u64 = 1000; // 1ms = 1kHz

// IPC channels
static MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

#[cfg(feature = "rc")]
static RC_MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

// GPS data channel (10Hz max)
#[cfg(feature = "gps")]
static GPS_DATA: Channel<CriticalSectionRawMutex, GpsData, 4> = Channel::new();

// Compass data channel (50Hz)
#[cfg(feature = "compass")]
static COMPASS_DATA: Channel<CriticalSectionRawMutex, CompassData, 8> = Channel::new();

// Navigation channels
#[cfg(feature = "nav")]
static NAV_CMD: Channel<CriticalSectionRawMutex, NavCommand, 4> = Channel::new();

#[cfg(feature = "nav")]
static NAV_MOTOR_CMD: Channel<CriticalSectionRawMutex, MotorCommand, 4> = Channel::new();

// Vehicle state machine channels
#[cfg(feature = "vehicle")]
static VEHICLE_CMD: Channel<CriticalSectionRawMutex, VehicleCommand, 4> = Channel::new();

// Navigation telemetry (when nav+wifi enabled)
#[cfg(all(feature = "nav", feature = "wifi"))]
static NAV_TELEMETRY: Channel<CriticalSectionRawMutex, NavTelemetry, 4> = Channel::new();

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

// Note: When using esp-wifi/esp-alloc feature, the heap is automatically
// initialized by esp-wifi. No manual heap initialization needed.

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Note: WiFi heap is automatically initialized by esp-wifi/esp-alloc feature

    // Initialize ESP-HAL
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize Embassy time driver
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Log boot information
    defmt::info!("================================");
    defmt::info!("    Ox Robotics Microkernel");
    defmt::info!("================================");

    let boot_info = BootInfo {
        chip: ox_chip::CHIP.name,
        control_rate_hz: (1_000_000 / CONTROL_PERIOD_US) as u32,
        #[cfg(feature = "wifi")]
        wifi_enabled: true,
        #[cfg(not(feature = "wifi"))]
        wifi_enabled: false,
        #[cfg(feature = "wifi")]
        wifi_ssid: Some(WIFI_SSID),
        #[cfg(not(feature = "wifi"))]
        wifi_ssid: None,
        #[cfg(feature = "ble")]
        ble_enabled: true,
        #[cfg(not(feature = "ble"))]
        ble_enabled: false,
        #[cfg(feature = "rc")]
        rc_enabled: true,
        #[cfg(not(feature = "rc"))]
        rc_enabled: false,
    };
    log_boot!(boot_info);

    // ========== GPIO Setup ==========
    let led_green = Output::new(peripherals.GPIO2, Level::Low);
    let led_red = Output::new(peripherals.GPIO3, Level::Low);
    let button = Input::new(peripherals.GPIO9, Pull::Up);

    log_gpio!(GpioConfig {
        status_led_pin: 2,
        fault_led_pin: 3,
        button_pin: 9,
    });

    // ========== CRSF/RC Setup ==========
    #[cfg(feature = "rc")]
    {
        log_rc_config!(RcConfig {
            protocol: RcProtocol::Crsf,
            rx_pin: 20,
            tx_pin: 21,
            baud_rate: 420_000,
        });

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
        log_start!(Component::Crsf);
    }

    // ========== Radio Setup (WiFi/BLE shared) ==========
    // ESP32 supports WiFi+BLE coexistence via time-multiplexing
    #[cfg(any(feature = "wifi", feature = "ble"))]
    let (radio_controller, mut rng) = {
        defmt::info!("radio: initializing shared controller");

        // Initialize RNG (needed for both WiFi and BLE)
        let rng = Rng::new(peripherals.RNG);

        // Use TIMG1 for radio (TIMG0 is used by Embassy time driver)
        let timg1 = TimerGroup::new(peripherals.TIMG1);

        // Initialize shared radio controller (supports WiFi + BLE coexistence)
        let init = mk_static!(
            EspWifiController<'static>,
            wifi_init(timg1.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
        );

        #[cfg(all(feature = "wifi", feature = "ble"))]
        defmt::info!("radio: wifi+ble coexistence enabled");

        (init, rng)
    };

    // ========== WiFi Setup ==========
    #[cfg(feature = "wifi")]
    {
        log_wifi!(WifiEvent { state: WifiState::Connecting, ssid: Some(WIFI_SSID) });

        // Create WiFi device in station mode
        let (wifi_interface, controller) =
            esp_wifi::wifi::new_with_mode(radio_controller, peripherals.WIFI, WifiStaDevice).unwrap();

        // Generate random seed for network stack
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        // Create network stack with DHCP
        let net_config = NetConfig::dhcpv4(Default::default());
        let (stack, runner) = embassy_net::new(
            wifi_interface,
            net_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );

        // Store stack reference for telemetry tasks
        let stack_ref = mk_static!(Stack<'static>, stack);

        // Spawn WiFi tasks
        spawner.spawn(wifi_connection_task(controller)).unwrap();
        spawner.spawn(wifi_net_task(runner)).unwrap();
        spawner.spawn(wifi_telemetry_task(stack_ref)).unwrap();

        log_start!(Component::Wifi);
    }

    // ========== BLE Setup ==========
    #[cfg(feature = "ble")]
    {
        defmt::info!("ble: initializing controller");

        // Create BLE connector (HCI interface) - shares radio with WiFi
        let connector = BleConnector::new(radio_controller, peripherals.BT);

        spawner.spawn(ble_controller_task(connector)).unwrap();
        log_start!(Component::Ble);
    }

    // Suppress unused variable warning when only one feature is enabled
    #[cfg(all(any(feature = "wifi", feature = "ble"), not(all(feature = "wifi", feature = "ble"))))]
    let _ = rng;

    // ========== GPS Setup ==========
    #[cfg(feature = "gps")]
    {
        defmt::info!("gps: configuring UART (GPIO 16 RX, GPIO 17 TX)");

        // GPS modules typically start at 9600 baud
        // After UBX config, we could switch to 115200 for higher throughput
        let uart_config = UartConfig::default().with_baudrate(9600);

        let uart = Uart::new(peripherals.UART0, uart_config)
            .unwrap()
            .with_rx(peripherals.GPIO16)
            .with_tx(peripherals.GPIO17)
            .into_async();

        let (rx, tx) = uart.split();

        spawner.spawn(gps_task(rx, tx)).unwrap();
        defmt::info!("gps: task spawned");
    }

    // ========== Compass Setup ==========
    #[cfg(feature = "compass")]
    {
        defmt::info!("compass: configuring I2C (GPIO 4 SDA, GPIO 5 SCL)");

        // I2C bus for magnetometer (shared with IMU)
        // QMC5883L runs at 400kHz max
        let i2c_config = esp_hal::i2c::master::Config::default()
            .with_frequency(400u32.kHz());

        let i2c = I2c::new(peripherals.I2C0, i2c_config)
            .unwrap()
            .with_sda(peripherals.GPIO4)
            .with_scl(peripherals.GPIO5)
            .into_async();

        spawner.spawn(compass_task(i2c)).unwrap();
        defmt::info!("compass: task spawned");
    }

    // ========== Navigation Setup ==========
    #[cfg(feature = "nav")]
    {
        spawner.spawn(nav_task()).unwrap();
        defmt::info!("nav: task spawned (10Hz, consumes GPS + compass)");
    }

    // ========== Vehicle State Machine Setup ==========
    #[cfg(feature = "vehicle")]
    {
        spawner.spawn(vehicle_task()).unwrap();
        defmt::info!("vehicle: state machine spawned");
    }

    // ========== Core Tasks ==========
    spawner.spawn(control_loop_task()).unwrap();
    spawner.spawn(status_led_task(led_green)).unwrap();
    spawner.spawn(fault_led_task(led_red)).unwrap();
    spawner.spawn(button_task(button)).unwrap();
    spawner.spawn(timing_report_task()).unwrap();

    defmt::info!("ready: all tasks spawned");
    defmt::info!("================================");
    defmt::info!("modes: Coast -> Velocity -> Position -> E-Stop");
    defmt::info!("================================");

    // Main supervisor loop
    loop {
        Timer::after(Duration::from_secs(10)).await;
        // Could add watchdog/health checks here
    }
}

/// Control loop task - runs motor and sensor servers at 1kHz
#[embassy_executor::task]
async fn control_loop_task() {
    log_start!(Component::Control);

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
            let event = match cmd {
                MotorCommand::SetVelocity { left, right } => MotorEvent::SetVelocity { left, right },
                MotorCommand::SetPosition { left, right } => MotorEvent::SetPosition { left, right },
                MotorCommand::Coast => MotorEvent::Coast,
                MotorCommand::EmergencyStop => MotorEvent::EmergencyStop,
            };
            log_motor!(Component::Button, event);
            motor_server.process_command(cmd);
        }

        // 2. Process RC commands (highest priority - manual override)
        #[cfg(feature = "rc")]
        if let Ok(cmd) = RC_MOTOR_CMD.try_receive() {
            motor_server.process_command(cmd);
        }

        // 3. Process NAV commands (lower priority than RC)
        #[cfg(feature = "nav")]
        if let Ok(cmd) = NAV_MOTOR_CMD.try_receive() {
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
            let event = match status.mode {
                ox_services::motor::ControlMode::Coast => MotorEvent::Coast,
                ox_services::motor::ControlMode::Velocity => MotorEvent::SetVelocity {
                    left: status.left_velocity,
                    right: status.right_velocity,
                },
                ox_services::motor::ControlMode::Position => MotorEvent::SetPosition {
                    left: status.left_position,
                    right: status.right_position,
                },
                ox_services::motor::ControlMode::Stopped => MotorEvent::EmergencyStop,
            };
            defmt::info!("ctrl: iteration={}, loop_us={}, mode={}", iteration, elapsed_us, event);
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

    log_start!(Component::Crsf);

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

                    // Log RC data
                    if decoder.link_quality() > 0 {
                        log_rc_data!(RcChannelData {
                            throttle: (throttle * 1000.0) as i16,
                            steering: (steering * 1000.0) as i16,
                            link_quality: decoder.link_quality(),
                            rssi: decoder.rssi(),
                            armed: rc_server.is_armed(),
                            failsafe: rc_server.is_failsafe(),
                        });
                    }
                }
            }
            Err(_e) => {
                log_warn!(Component::Crsf, "uart_error");
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
    log_start!(Component::Led);

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
    log_start!(Component::Button);

    let mut last_state = true;
    let mut mode_index = 0u8;

    let modes = [
        MotorCommand::Coast,
        MotorCommand::SetVelocity { left: 500, right: 500 },
        MotorCommand::SetPosition { left: 1000, right: 1000 },
        MotorCommand::EmergencyStop,
    ];

    let events = [
        MotorEvent::Coast,
        MotorEvent::SetVelocity { left: 500, right: 500 },
        MotorEvent::SetPosition { left: 1000, right: 1000 },
        MotorEvent::EmergencyStop,
    ];

    loop {
        let current = button.is_high();

        // Detect falling edge
        if last_state && !current {
            mode_index = (mode_index + 1) % modes.len() as u8;
            let cmd = modes[mode_index as usize];

            log_button!(ButtonEvent { command: events[mode_index as usize] });
            let _ = MOTOR_CMD.try_send(cmd);
        }
        last_state = current;

        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Timing report task - prints control loop statistics
#[embassy_executor::task]
async fn timing_report_task() {
    log_start!(Component::Timing);

    // Wait for control loop to stabilize
    Timer::after(Duration::from_secs(2)).await;

    loop {
        Timer::after(Duration::from_secs(5)).await;

        let mut stats = TIMING_STATS.lock().await;

        if stats.count > 0 {
            let meets_target = stats.max_us < CONTROL_PERIOD_US;

            log_timing!(logging::TimingStats {
                count: stats.count,
                min_us: stats.min_us as u32,
                max_us: stats.max_us as u32,
                avg_us: stats.avg_us() as u32,
                target_us: CONTROL_PERIOD_US as u32,
                within_budget: meets_target,
            });

            // Reset stats for next period
            stats.reset();
        }
    }
}

// ========== WiFi Tasks ==========

/// WiFi connection management task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_connection_task(mut controller: WifiController<'static>) {
    log_start!(Component::Wifi);

    loop {
        // Wait for WiFi to be ready
        if matches!(esp_wifi::wifi::wifi_state(), esp_wifi::wifi::WifiState::StaConnected) {
            // Already connected, wait for disconnect event
            controller.wait_for_event(EspWifiEvent::StaDisconnected).await;
            log_wifi!(WifiEvent { state: WifiState::Disconnected, ssid: Some(WIFI_SSID) });
            Timer::after(Duration::from_secs(1)).await;
        }

        // Configure WiFi
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: WIFI_SSID.try_into().unwrap(),
                password: WIFI_PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            defmt::debug!("wifi: starting");
            controller.start_async().await.unwrap();
        }

        // Connect to AP
        log_wifi!(WifiEvent { state: WifiState::Connecting, ssid: Some(WIFI_SSID) });
        match controller.connect_async().await {
            Ok(_) => log_wifi!(WifiEvent { state: WifiState::Connected, ssid: Some(WIFI_SSID) }),
            Err(_e) => {
                log_wifi!(WifiEvent { state: WifiState::Error, ssid: Some(WIFI_SSID) });
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

/// WiFi network stack runner task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    defmt::debug!("wifi: network stack running");
    runner.run().await;
}

/// WiFi telemetry streaming task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_telemetry_task(stack: &'static Stack<'static>) {
    defmt::debug!("wifi: telemetry task starting");

    // Create comms server
    let mut config = CommsConfig::default();
    let _ = config.wifi.ssid.push_str(WIFI_SSID);
    let mut comms = CommsServer::new(config);

    // Wait for network to be up
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    defmt::debug!("wifi: link up");

    // Wait for IP address
    loop {
        if let Some(cfg) = stack.config_v4() {
            let ip = cfg.address.address().octets();
            log_wifi!(WifiEvent {
                state: WifiState::GotIp { ip },
                ssid: Some(WIFI_SSID),
            });
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    comms.start_connect();
    comms.on_connected();
    defmt::info!("wifi: telemetry ready");

    // Rate limiting for nav telemetry (5Hz = 200ms interval)
    #[cfg(feature = "nav")]
    const NAV_TELEMETRY_INTERVAL_MS: u64 = 200;
    #[cfg(feature = "nav")]
    let mut last_nav_telemetry = Instant::now();
    #[cfg(feature = "nav")]
    let mut nav_telem_count = 0u32;

    // Telemetry loop
    let mut iteration = 0u32;

    loop {
        // Update comms server state
        let timestamp = Instant::now().as_millis() as u32;
        comms.update(timestamp);

        // Consume nav telemetry with rate limiting
        #[cfg(feature = "nav")]
        {
            // Drain channel but only process at rate limit
            while let Ok(nav_telem) = NAV_TELEMETRY.try_receive() {
                let now = Instant::now();
                if now.duration_since(last_nav_telemetry).as_millis() >= NAV_TELEMETRY_INTERVAL_MS {
                    // Create telemetry message from nav data
                    let mut msg = TelemetryMessage::new(TelemetryType::Nav, timestamp);
                    let bytes = nav_telem.to_bytes();
                    let _ = msg.payload.extend_from_slice(&bytes);

                    // Queue for sending (ignore errors if buffer full)
                    let _ = comms.send_telemetry(msg);
                    last_nav_telemetry = now;
                    nav_telem_count += 1;
                }
                // Older messages are dropped (rate limiting)
            }
        }

        // Log status periodically
        iteration += 1;
        if iteration % 100 == 0 {
            let stats = comms.stats();
            #[cfg(feature = "nav")]
            defmt::info!("wifi: tx={}, rx={}, nav_telem={}", stats.tx_count, stats.rx_count, nav_telem_count);
            #[cfg(not(feature = "nav"))]
            defmt::info!("wifi: tx={}, rx={}, connected={}", stats.tx_count, stats.rx_count, comms.is_connected());
        }

        // Send queued telemetry messages
        while let Some(msg) = comms.pop_telemetry() {
            // In a real implementation, this would send over a socket
            defmt::debug!("wifi: telemetry type={}, ts={}", msg.timestamp_ms, msg.timestamp_ms);
            comms.on_message_sent();
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

// ========== BLE Tasks ==========

/// BLE controller task - handles HCI communication
///
/// This provides low-level HCI access to the BLE controller.
/// For full GATT peripheral mode, a higher-level library like
/// `bleps` or `trouble` would be needed on top of this HCI layer.
#[cfg(feature = "ble")]
#[embassy_executor::task]
async fn ble_controller_task(_connector: BleConnector<'static>) {
    use esp_wifi::ble::{have_hci_read_data, read_hci};

    log_start!(Component::Ble);
    defmt::info!("ble: HCI interface ready (GATT requires bleps/trouble)");

    // BLE HCI event loop
    let mut iteration = 0u32;
    let mut hci_events = 0u32;
    let mut hci_buffer = [0u8; 256];

    loop {
        // Check for incoming HCI data
        if have_hci_read_data() {
            // Read HCI packet into buffer
            let len = read_hci(&mut hci_buffer);
            if len > 0 {
                hci_events += 1;
                defmt::debug!("ble: hci_packet bytes={}, total={}", len, hci_events);
            }
        }

        // Log status periodically
        iteration += 1;
        if iteration % 100 == 0 {
            log_ble!(BleEvent {
                hci_events,
                connected: false, // TODO: track actual connection state
            });
        }

        // Yield to other tasks
        Timer::after(Duration::from_millis(10)).await;
    }
}

// ========== GPS Tasks ==========

/// GPS receiver task - reads UART and parses NMEA/UBX
///
/// Wiring:
/// - GPIO 16: GPS TX -> ESP RX
/// - GPIO 17: GPS RX <- ESP TX (for UBX config commands)
#[cfg(feature = "gps")]
#[embassy_executor::task]
async fn gps_task(mut rx: UartRx<'static, esp_hal::Async>, mut tx: UartTx<'static, esp_hal::Async>) {
    use embedded_io_async::{Read, Write};

    defmt::info!("[GPS] Starting GPS receiver task");

    // Configure GPS for 10Hz output via UBX command
    let mut cfg_buf = [0u8; 16];
    let len = UbxConfigBuilder::build_cfg_rate(100, &mut cfg_buf); // 100ms = 10Hz
    if tx.write_all(&cfg_buf[..len]).await.is_err() {
        defmt::warn!("[GPS] Failed to send UBX config");
    }

    // Small delay for GPS to process config
    Timer::after(Duration::from_millis(100)).await;

    let mut server = GpsServer::new(GpsConfig::default());
    let mut buf = [0u8; 1];
    let mut last_log = Instant::now();

    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                if let Some(data) = server.process_byte(buf[0]) {
                    // Publish to IPC channel
                    let _ = GPS_DATA.try_send(data);

                    // Log periodically (not every update)
                    if last_log.elapsed() > Duration::from_secs(5) {
                        if data.valid {
                            defmt::info!(
                                "[GPS] lat={} lon={} alt={}m spd={}m/s sats={} fix={}",
                                data.latitude as f32,
                                data.longitude as f32,
                                data.altitude,
                                data.speed,
                                data.satellites,
                                data.fix as u8
                            );
                        } else {
                            defmt::warn!("[GPS] No fix, sats={}", data.satellites);
                        }
                        last_log = Instant::now();
                    }
                }
            }
            Err(_) => {
                defmt::warn!("[GPS] UART read error");
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

// ========== Compass Tasks ==========

/// Compass task - reads I2C magnetometer and computes heading
///
/// Wiring (shared with IMU):
/// - GPIO 4: I2C SDA
/// - GPIO 5: I2C SCL
#[cfg(feature = "compass")]
#[embassy_executor::task]
async fn compass_task(i2c: I2c<'static, esp_hal::Async>) {
    defmt::info!("[COMPASS] Starting compass task");

    // Local magnetic declination (update for your location!)
    // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    let config = CompassConfig {
        declination: -12.0, // Example: -12 degrees west
        min_cal_samples: 200,
    };
    let mut server = CompassServer::new(config);

    // Initialize QMC5883L
    let mut mag = Qmc5883l::new(i2c);
    if mag.init().is_err() {
        defmt::error!("[COMPASS] Init failed");
        return;
    }

    defmt::info!("[COMPASS] QMC5883L initialized");

    let mut last_log = Instant::now();
    let period = Duration::from_millis(20); // 50Hz

    loop {
        match mag.read() {
            Ok(raw) => {
                let data = server.process(raw);

                // Publish to IPC channel
                let _ = COMPASS_DATA.try_send(data);

                // Log periodically
                if last_log.elapsed() > Duration::from_secs(5) {
                    let cal_status = match data.calibration {
                        ox_services::compass::CalibrationStatus::Uncalibrated => "uncal",
                        ox_services::compass::CalibrationStatus::Collecting => "collecting",
                        ox_services::compass::CalibrationStatus::Calibrated => "cal",
                    };
                    defmt::info!(
                        "[COMPASS] heading={} true={} cal={}",
                        data.heading as i32,
                        data.heading_true as i32,
                        cal_status
                    );
                    last_log = Instant::now();
                }
            }
            Err(_) => {
                // NotReady is normal, just means no new data yet
            }
        }

        Timer::after(period).await;
    }
}

// ========== Navigation Tasks ==========

/// Navigation task - runs waypoint following at 10Hz
///
/// Consumes GPS and compass data, produces motor commands.
/// Priority: RC > NAV > Button
#[cfg(feature = "nav")]
#[embassy_executor::task]
async fn nav_task() {
    defmt::info!("[NAV] Starting navigation task");

    let mut server = NavServer::new(NavConfig::default());
    let period = Duration::from_millis(100); // 10Hz
    let mut last_update = Instant::now();
    let mut last_log = Instant::now();

    loop {
        // Calculate elapsed time
        let now = Instant::now();
        let elapsed_ms = now.duration_since(last_update).as_millis() as u32;
        last_update = now;

        // Consume GPS data (non-blocking)
        while let Ok(gps) = GPS_DATA.try_receive() {
            server.process_gps(gps);
        }

        // Consume compass data (non-blocking)
        while let Ok(compass) = COMPASS_DATA.try_receive() {
            server.process_compass(compass);
        }

        // Process external commands (non-blocking)
        while let Ok(cmd) = NAV_CMD.try_receive() {
            defmt::info!("[NAV] Received command");
            server.process_command(cmd);
        }

        // Run navigation update
        if let Some(motor_cmd) = server.update(elapsed_ms) {
            let _ = NAV_MOTOR_CMD.try_send(motor_cmd);
        }

        // Publish telemetry (when WiFi enabled)
        #[cfg(feature = "wifi")]
        {
            let timestamp = now.as_millis() as u32;
            // Vehicle mode 0 = no vehicle state machine, just nav
            let telem = server.telemetry(0, timestamp);
            let _ = NAV_TELEMETRY.try_send(telem);
        }

        // Log status periodically
        if last_log.elapsed() > Duration::from_secs(5) {
            let status = server.status();
            defmt::info!(
                "[NAV] state={} wp={}/{} dist={}m err={}deg home={}",
                status.state as u8,
                status.waypoint_index,
                status.waypoint_count,
                status.distance_to_waypoint_m as i32,
                status.heading_error_deg as i32,
                if status.home_valid { "yes" } else { "no" }
            );
            last_log = Instant::now();
        }

        Timer::after(period).await;
    }
}

// ========== Vehicle State Machine Task ==========

/// Vehicle state machine task - manages vehicle mode and arming
///
/// Processes commands from RC/WiFi and coordinates with NavServer.
/// Runs at 20Hz to handle mode transitions quickly.
#[cfg(feature = "vehicle")]
#[embassy_executor::task]
async fn vehicle_task() {
    defmt::info!("[VEHICLE] Starting vehicle state machine");

    let mut vsm = VehicleStateMachine::new();
    let period = Duration::from_millis(50); // 20Hz
    let mut last_log = Instant::now();

    // Initial state
    vsm.set_no_faults(true);

    loop {
        // Process vehicle commands (from RC or WiFi)
        while let Ok(cmd) = VEHICLE_CMD.try_receive() {
            let result = vsm.process_command(cmd);
            match result {
                TransitionResult::Ok => {
                    defmt::info!("[VEHICLE] Mode changed to {}", vsm.mode() as u8);

                    // Handle mode-specific actions
                    #[cfg(feature = "nav")]
                    match vsm.mode() {
                        VehicleMode::Auto => {
                            let _ = NAV_CMD.try_send(NavCommand::Start);
                        }
                        VehicleMode::Rtl => {
                            let _ = NAV_CMD.try_send(NavCommand::ReturnToHome);
                        }
                        VehicleMode::Disarmed | VehicleMode::Emergency => {
                            let _ = NAV_CMD.try_send(NavCommand::Pause);
                        }
                        _ => {}
                    }
                }
                TransitionResult::ArmingChecksFailed => {
                    defmt::warn!("[VEHICLE] Arming checks failed: {}/{}",
                        vsm.checks().passing_count(),
                        ox_services::vehicle::ArmingChecks::total_checks()
                    );
                }
                TransitionResult::NoMission => {
                    defmt::warn!("[VEHICLE] No mission loaded");
                }
                TransitionResult::NoHome => {
                    defmt::warn!("[VEHICLE] No home position set");
                }
                _ => {
                    defmt::warn!("[VEHICLE] Command rejected");
                }
            }
        }

        // Update timing
        vsm.update(50);

        // Update arming checks from sensor data
        #[cfg(feature = "gps")]
        {
            // Check if GPS data is fresh (would need GPS channel here)
            // For now, this would be updated by nav_task
        }

        // Log status periodically
        if last_log.elapsed() > Duration::from_secs(5) {
            defmt::info!(
                "[VEHICLE] mode={} checks={}/{} time={}s",
                vsm.mode() as u8,
                vsm.checks().passing_count(),
                ox_services::vehicle::ArmingChecks::total_checks(),
                vsm.time_in_mode_ms() / 1000
            );
            last_log = Instant::now();
        }

        Timer::after(period).await;
    }
}
