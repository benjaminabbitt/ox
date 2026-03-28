//! Structured logging events
//!
//! Defines typed events for all system components.
//! Uses defmt for efficient binary encoding.

use defmt::Format;

/// System boot information
#[derive(Format)]
pub struct BootInfo<'a> {
    pub chip: &'a str,
    pub control_rate_hz: u32,
    pub wifi_enabled: bool,
    pub wifi_ssid: Option<&'a str>,
    pub ble_enabled: bool,
    pub rc_enabled: bool,
}

/// GPIO pin configuration
#[derive(Format)]
pub struct GpioConfig {
    pub status_led_pin: u8,
    pub fault_led_pin: u8,
    pub button_pin: u8,
}

/// RC receiver configuration
#[derive(Format)]
pub struct RcConfig {
    pub protocol: RcProtocol,
    pub rx_pin: u8,
    pub tx_pin: u8,
    pub baud_rate: u32,
}

#[derive(Format, Clone, Copy)]
pub enum RcProtocol {
    Crsf,
    Sbus,
}

/// RC channel data
#[derive(Format)]
pub struct RcChannelData {
    pub throttle: i16,     // Scaled to -1000..1000
    pub steering: i16,     // Scaled to -1000..1000
    pub link_quality: u8,
    pub rssi: u8,
    pub armed: bool,
    pub failsafe: bool,
}

/// Motor command event
#[derive(Format, Clone, Copy)]
pub enum MotorEvent {
    SetVelocity { left: i16, right: i16 },
    SetPosition { left: i32, right: i32 },
    Coast,
    EmergencyStop,
}

/// Control loop timing statistics
#[derive(Format)]
pub struct TimingStats {
    pub count: u32,
    pub min_us: u32,
    pub max_us: u32,
    pub avg_us: u32,
    pub target_us: u32,
    pub within_budget: bool,
}

/// WiFi connection state
#[derive(Format, Clone, Copy)]
pub enum WifiState {
    Disconnected,
    Connecting,
    Connected,
    GotIp { ip: [u8; 4] },
    Error,
}

/// WiFi event
#[derive(Format)]
pub struct WifiEvent<'a> {
    pub state: WifiState,
    pub ssid: Option<&'a str>,
}

/// Telemetry stats
#[derive(Format)]
pub struct TelemetryStats {
    pub uptime_secs: u32,
    pub control_loops: u32,
    pub motor_cmd: MotorEvent,
    pub wifi_connected: bool,
    pub rc_armed: bool,
    pub rc_failsafe: bool,
}

/// BLE event
#[derive(Format)]
pub struct BleEvent {
    pub hci_events: u32,
    pub connected: bool,
}

/// UART/serial error
#[derive(Format)]
pub struct UartError {
    pub component: Component,
    pub error_code: u8,
}

/// System component identifier
#[derive(Format, Clone, Copy)]
pub enum Component {
    Control,
    Crsf,
    Wifi,
    Ble,
    Motor,
    Button,
    Led,
    Timing,
}

/// Button press event
#[derive(Format)]
pub struct ButtonEvent {
    pub command: MotorEvent,
}

// ============================================================
// Logging macros that use structured types
// ============================================================

/// Log system boot
#[macro_export]
macro_rules! log_boot {
    ($info:expr) => {
        defmt::info!("boot: {}", $info)
    };
}

/// Log GPIO configuration
#[macro_export]
macro_rules! log_gpio {
    ($config:expr) => {
        defmt::info!("gpio: {}", $config)
    };
}

/// Log RC configuration
#[macro_export]
macro_rules! log_rc_config {
    ($config:expr) => {
        defmt::info!("rc_config: {}", $config)
    };
}

/// Log RC channel data
#[macro_export]
macro_rules! log_rc_data {
    ($data:expr) => {
        defmt::debug!("rc: {}", $data)
    };
}

/// Log motor event
#[macro_export]
macro_rules! log_motor {
    ($component:expr, $event:expr) => {
        defmt::info!("motor: component={}, event={}", $component, $event)
    };
}

/// Log timing stats
#[macro_export]
macro_rules! log_timing {
    ($stats:expr) => {
        defmt::info!("timing: {}", $stats)
    };
}

/// Log WiFi event
#[macro_export]
macro_rules! log_wifi {
    ($event:expr) => {
        defmt::info!("wifi: {}", $event)
    };
}

/// Log telemetry
#[macro_export]
macro_rules! log_telemetry {
    ($stats:expr) => {
        defmt::info!("telemetry: {}", $stats)
    };
}

/// Log BLE event
#[macro_export]
macro_rules! log_ble {
    ($event:expr) => {
        defmt::info!("ble: {}", $event)
    };
}

/// Log button event
#[macro_export]
macro_rules! log_button {
    ($event:expr) => {
        defmt::info!("button: {}", $event)
    };
}

/// Log error
#[macro_export]
macro_rules! log_error {
    ($component:expr, $msg:expr) => {
        defmt::error!("{}: {}", $component, $msg)
    };
}

/// Log warning
#[macro_export]
macro_rules! log_warn {
    ($component:expr, $msg:expr) => {
        defmt::warn!("{}: {}", $component, $msg)
    };
}

/// Log component start
#[macro_export]
macro_rules! log_start {
    ($component:expr) => {
        defmt::info!("start: {}", $component)
    };
}
