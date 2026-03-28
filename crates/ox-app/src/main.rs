//! Ox Robotics Microkernel
//!
//! Features:
//! - 1kHz control loop (<1ms latency)
//! - Motor and sensor server integration
//! - WiFi telemetry (feature: wifi)
//! - BLE controller input (feature: ble) - HCI layer initialized
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

// WiFi imports
#[cfg(feature = "wifi")]
use {
    embassy_net::{Config as NetConfig, Runner, Stack, StackResources},
    esp_wifi::wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState,
    },
    ox_services::comms::{CommsConfig, CommsServer},
};

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

    // Initialize logging
    esp_println::logger::init_logger_from_env();

    log::info!("================================");
    log::info!("    Ox Robotics Microkernel");
    log::info!("================================");
    log::info!("Chip: {}", ox_chip::CHIP.name);
    log::info!("Target control rate: {} Hz", 1_000_000 / CONTROL_PERIOD_US);

    // Feature flags
    #[cfg(feature = "wifi")]
    log::info!("WiFi: ENABLED (SSID: {})", WIFI_SSID);
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

    // ========== WiFi Setup ==========
    #[cfg(feature = "wifi")]
    {
        log::info!("[WIFI] Initializing WiFi stack...");

        // Initialize RNG for network seed
        let mut rng = Rng::new(peripherals.RNG);

        // Use TIMG1 for WiFi (TIMG0 is used by Embassy time driver)
        let timg1 = TimerGroup::new(peripherals.TIMG1);

        // Initialize WiFi controller (requires 'static lifetime)
        let wifi_init = mk_static!(
            EspWifiController<'static>,
            wifi_init(timg1.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
        );

        // Create WiFi device in station mode
        let (wifi_interface, controller) =
            esp_wifi::wifi::new_with_mode(wifi_init, peripherals.WIFI, WifiStaDevice).unwrap();

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

        log::info!("[WIFI] WiFi tasks spawned");
    }

    // ========== BLE Setup ==========
    // Note: BLE-only mode (when wifi feature is disabled)
    // WiFi and BLE can coexist but require shared radio initialization
    #[cfg(all(feature = "ble", not(feature = "wifi")))]
    {
        log::info!("[BLE] Initializing BLE controller...");

        // Initialize RNG
        let rng = Rng::new(peripherals.RNG);

        // Use TIMG1 for BLE (TIMG0 is used by Embassy time driver)
        let timg1 = TimerGroup::new(peripherals.TIMG1);

        // Initialize BLE controller (shares radio with WiFi when both enabled)
        let ble_init = mk_static!(
            EspWifiController<'static>,
            wifi_init(timg1.timer0, rng, peripherals.RADIO_CLK).unwrap()
        );

        // Create BLE connector (HCI interface)
        let connector = BleConnector::new(ble_init, peripherals.BT);

        spawner.spawn(ble_controller_task(connector)).unwrap();
        log::info!("[BLE] BLE controller task spawned");
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

// ========== WiFi Tasks ==========

/// WiFi connection management task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_connection_task(mut controller: WifiController<'static>) {
    log::info!("[WIFI] Starting connection task");

    loop {
        // Wait for WiFi to be ready
        if matches!(esp_wifi::wifi::wifi_state(), WifiState::StaConnected) {
            // Already connected, wait for disconnect event
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            log::warn!("[WIFI] Disconnected from AP");
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
            log::info!("[WIFI] Starting WiFi...");
            controller.start_async().await.unwrap();
            log::info!("[WIFI] WiFi started");
        }

        // Connect to AP
        log::info!("[WIFI] Connecting to {}...", WIFI_SSID);
        match controller.connect_async().await {
            Ok(_) => log::info!("[WIFI] Connected to AP"),
            Err(e) => {
                log::warn!("[WIFI] Connection failed: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

/// WiFi network stack runner task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    log::info!("[WIFI] Starting network stack");
    runner.run().await;
}

/// WiFi telemetry streaming task
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn wifi_telemetry_task(stack: &'static Stack<'static>) {
    log::info!("[WIFI] Starting telemetry task");

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
    log::info!("[WIFI] Link is up");

    // Wait for IP address
    loop {
        if let Some(cfg) = stack.config_v4() {
            log::info!("[WIFI] Got IP: {}", cfg.address.address());
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    comms.start_connect();
    comms.on_connected();
    log::info!("[WIFI] Telemetry ready");

    // Telemetry loop
    let mut iteration = 0u32;

    loop {
        // Update comms server state
        let timestamp = Instant::now().as_millis() as u32;
        comms.update(timestamp);

        // Log status periodically
        iteration += 1;
        if iteration % 100 == 0 {
            let stats = comms.stats();
            log::info!(
                "[WIFI] State: {:?}, TX: {}, RX: {}",
                comms.state(),
                stats.tx_count,
                stats.rx_count
            );
        }

        // Send queued telemetry messages
        while let Some(msg) = comms.pop_telemetry() {
            // In a real implementation, this would send over a socket
            // For now, just log it
            log::debug!(
                "[WIFI] Telemetry: {:?} @ {}ms",
                msg.msg_type,
                msg.timestamp_ms
            );
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

    log::info!("[BLE] Starting BLE controller task");
    log::info!("[BLE] HCI interface ready");
    log::info!("[BLE] Note: Full GATT peripheral requires additional library (bleps/trouble)");

    // BLE HCI event loop
    // In a full implementation, this would:
    // 1. Send HCI commands to set up advertising
    // 2. Handle connection events
    // 3. Process GATT read/write requests
    //
    // For now, we just log HCI activity to show the BLE stack is working

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
                log::debug!(
                    "[BLE] HCI packet received: {} bytes, total events: {}",
                    len,
                    hci_events
                );
            }
        }

        // Log status periodically
        iteration += 1;
        if iteration % 100 == 0 {
            log::info!("[BLE] Controller running, HCI events: {}", hci_events);
        }

        // Yield to other tasks
        Timer::after(Duration::from_millis(10)).await;
    }
}
