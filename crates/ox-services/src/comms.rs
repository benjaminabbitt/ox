//! Communications server
//!
//! Handles WiFi/BLE connectivity and telemetry streaming.
//!
//! Features:
//! - Connection state management
//! - Telemetry message formatting
//! - Remote command handling
//! - Automatic reconnection
//!
//! Architecture:
//! - CommsServer is hardware-agnostic (testable on host)
//! - Network driver trait abstracts WiFi/BLE/mock
//! - Runs at lowest priority (Thread executor on C3)

use heapless::{String, Vec};

/// Maximum telemetry message size
pub const MAX_TELEMETRY_SIZE: usize = 256;

/// Maximum command size
pub const MAX_COMMAND_SIZE: usize = 64;

/// Connection state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Not connected, not trying
    Disconnected,
    /// Attempting to connect
    Connecting,
    /// Connected and ready
    Connected,
    /// Connection failed, will retry
    Reconnecting,
    /// Fatal error, manual intervention needed
    Error,
}

impl Default for ConnectionState {
    fn default() -> Self {
        ConnectionState::Disconnected
    }
}

/// WiFi configuration
#[derive(Debug, Clone)]
pub struct WifiConfig {
    /// SSID (network name)
    pub ssid: String<32>,
    /// Password (WPA2)
    pub password: String<64>,
    /// Use DHCP for IP assignment
    pub use_dhcp: bool,
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            ssid: String::new(),
            password: String::new(),
            use_dhcp: true,
        }
    }
}

/// Telemetry packet types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryType {
    /// Motor status (velocity, position, fault)
    Motor,
    /// IMU data (accel, gyro, attitude)
    Imu,
    /// Pose (x, y, theta)
    Pose,
    /// System status (timing, memory, uptime)
    System,
    /// Heartbeat (connection keepalive)
    Heartbeat,
}

/// Telemetry message
#[derive(Debug, Clone)]
pub struct TelemetryMessage {
    /// Message type
    pub msg_type: TelemetryType,
    /// Timestamp (ms since boot)
    pub timestamp_ms: u32,
    /// Payload data (JSON or binary)
    pub payload: Vec<u8, MAX_TELEMETRY_SIZE>,
}

impl TelemetryMessage {
    /// Create a new telemetry message
    pub fn new(msg_type: TelemetryType, timestamp_ms: u32) -> Self {
        Self {
            msg_type,
            timestamp_ms,
            payload: Vec::new(),
        }
    }

    /// Create heartbeat message
    pub fn heartbeat(timestamp_ms: u32) -> Self {
        let mut msg = Self::new(TelemetryType::Heartbeat, timestamp_ms);
        // Simple heartbeat payload
        let _ = msg.payload.extend_from_slice(b"OK");
        msg
    }
}

/// Remote command types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemoteCommand {
    /// Set motor velocity
    SetVelocity { left: i16, right: i16 },
    /// Set motor position
    SetPosition { left: i32, right: i32 },
    /// Emergency stop
    EmergencyStop,
    /// Coast motors
    Coast,
    /// Reset pose to origin
    ResetPose,
    /// Request telemetry burst
    RequestTelemetry { msg_type: TelemetryType },
    /// Ping (connection test)
    Ping,
}

/// Comms server error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommsError {
    /// Not connected
    NotConnected,
    /// Send buffer full
    BufferFull,
    /// Invalid command
    InvalidCommand,
    /// Network error
    NetworkError,
    /// Timeout
    Timeout,
}

/// Comms server statistics
#[derive(Debug, Clone, Copy, Default)]
pub struct CommsStats {
    /// Messages sent
    pub tx_count: u32,
    /// Messages received
    pub rx_count: u32,
    /// Errors encountered
    pub error_count: u32,
    /// Reconnection attempts
    pub reconnect_count: u32,
    /// Time connected (ms)
    pub connected_time_ms: u32,
}

/// Comms server configuration
#[derive(Debug, Clone)]
pub struct CommsConfig {
    /// WiFi configuration
    pub wifi: WifiConfig,
    /// Telemetry send rate (Hz)
    pub telemetry_rate_hz: u8,
    /// Heartbeat interval (ms)
    pub heartbeat_interval_ms: u32,
    /// Reconnect delay (ms)
    pub reconnect_delay_ms: u32,
    /// Connection timeout (ms)
    pub connect_timeout_ms: u32,
}

impl Default for CommsConfig {
    fn default() -> Self {
        Self {
            wifi: WifiConfig::default(),
            telemetry_rate_hz: 10,
            heartbeat_interval_ms: 1000,
            reconnect_delay_ms: 5000,
            connect_timeout_ms: 10000,
        }
    }
}

/// Communications server
///
/// Manages WiFi/BLE connectivity and telemetry.
/// Hardware-agnostic core logic (actual networking in ox-app).
pub struct CommsServer {
    config: CommsConfig,
    state: ConnectionState,
    stats: CommsStats,
    last_heartbeat_ms: u32,
    tx_queue: Vec<TelemetryMessage, 8>,
    rx_queue: Vec<RemoteCommand, 8>,
}

impl CommsServer {
    /// Create a new comms server
    pub fn new(config: CommsConfig) -> Self {
        Self {
            config,
            state: ConnectionState::Disconnected,
            stats: CommsStats::default(),
            last_heartbeat_ms: 0,
            tx_queue: Vec::new(),
            rx_queue: Vec::new(),
        }
    }

    /// Get current connection state
    pub fn state(&self) -> ConnectionState {
        self.state
    }

    /// Get statistics
    pub fn stats(&self) -> CommsStats {
        self.stats
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.state == ConnectionState::Connected
    }

    /// Notify that connection was established
    pub fn on_connected(&mut self) {
        self.state = ConnectionState::Connected;
        self.stats.reconnect_count = self.stats.reconnect_count.saturating_add(
            if self.stats.tx_count > 0 { 1 } else { 0 }
        );
    }

    /// Notify that connection was lost
    pub fn on_disconnected(&mut self) {
        if self.state == ConnectionState::Connected {
            self.state = ConnectionState::Reconnecting;
        }
    }

    /// Notify connection error
    pub fn on_error(&mut self) {
        self.state = ConnectionState::Error;
        self.stats.error_count = self.stats.error_count.saturating_add(1);
    }

    /// Start connecting
    pub fn start_connect(&mut self) {
        if self.state == ConnectionState::Disconnected
            || self.state == ConnectionState::Reconnecting
        {
            self.state = ConnectionState::Connecting;
        }
    }

    /// Queue telemetry message for sending
    pub fn send_telemetry(&mut self, msg: TelemetryMessage) -> Result<(), CommsError> {
        if !self.is_connected() {
            return Err(CommsError::NotConnected);
        }
        self.tx_queue.push(msg).map_err(|_| CommsError::BufferFull)
    }

    /// Get next telemetry message to send
    pub fn pop_telemetry(&mut self) -> Option<TelemetryMessage> {
        if self.tx_queue.is_empty() {
            None
        } else {
            Some(self.tx_queue.remove(0))
        }
    }

    /// Queue received command
    pub fn receive_command(&mut self, cmd: RemoteCommand) -> Result<(), CommsError> {
        self.rx_queue.push(cmd).map_err(|_| CommsError::BufferFull)?;
        self.stats.rx_count = self.stats.rx_count.saturating_add(1);
        Ok(())
    }

    /// Get next command from queue
    pub fn pop_command(&mut self) -> Option<RemoteCommand> {
        if self.rx_queue.is_empty() {
            None
        } else {
            Some(self.rx_queue.remove(0))
        }
    }

    /// Update server (call periodically)
    pub fn update(&mut self, current_time_ms: u32) {
        // Update connected time
        if self.is_connected() {
            self.stats.connected_time_ms = self.stats.connected_time_ms.saturating_add(1);
        }

        // Check if heartbeat needed
        if self.is_connected() {
            let elapsed = current_time_ms.wrapping_sub(self.last_heartbeat_ms);
            if elapsed >= self.config.heartbeat_interval_ms {
                let hb = TelemetryMessage::heartbeat(current_time_ms);
                let _ = self.send_telemetry(hb);
                self.last_heartbeat_ms = current_time_ms;
            }
        }
    }

    /// Mark message as sent (called after successful network send)
    pub fn on_message_sent(&mut self) {
        self.stats.tx_count = self.stats.tx_count.saturating_add(1);
    }

    /// Get telemetry queue length
    pub fn tx_queue_len(&self) -> usize {
        self.tx_queue.len()
    }

    /// Get command queue length
    pub fn rx_queue_len(&self) -> usize {
        self.rx_queue.len()
    }

    /// Clear all queues
    pub fn clear_queues(&mut self) {
        self.tx_queue.clear();
        self.rx_queue.clear();
    }

    /// Get WiFi SSID (for display)
    pub fn ssid(&self) -> &str {
        &self.config.wifi.ssid
    }
}

impl Default for CommsServer {
    fn default() -> Self {
        Self::new(CommsConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_server() -> CommsServer {
        CommsServer::default()
    }

    fn make_config(ssid: &str) -> CommsConfig {
        let mut config = CommsConfig::default();
        config.wifi.ssid = String::try_from(ssid).unwrap();
        config
    }

    // === Connection State Tests ===

    #[test]
    fn starts_disconnected() {
        let server = make_server();
        assert_eq!(server.state(), ConnectionState::Disconnected);
        assert!(!server.is_connected());
    }

    #[test]
    fn start_connect_changes_state() {
        let mut server = make_server();
        server.start_connect();
        assert_eq!(server.state(), ConnectionState::Connecting);
    }

    #[test]
    fn on_connected_changes_state() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();
        assert_eq!(server.state(), ConnectionState::Connected);
        assert!(server.is_connected());
    }

    #[test]
    fn on_disconnected_triggers_reconnect() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();
        server.on_disconnected();
        assert_eq!(server.state(), ConnectionState::Reconnecting);
    }

    #[test]
    fn on_error_sets_error_state() {
        let mut server = make_server();
        server.on_error();
        assert_eq!(server.state(), ConnectionState::Error);
        assert_eq!(server.stats().error_count, 1);
    }

    #[test]
    fn reconnect_from_reconnecting_state() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();
        server.on_disconnected();
        assert_eq!(server.state(), ConnectionState::Reconnecting);
        server.start_connect();
        assert_eq!(server.state(), ConnectionState::Connecting);
    }

    // === Telemetry Tests ===

    #[test]
    fn send_telemetry_requires_connection() {
        let mut server = make_server();
        let msg = TelemetryMessage::heartbeat(1000);
        assert_eq!(server.send_telemetry(msg), Err(CommsError::NotConnected));
    }

    #[test]
    fn send_telemetry_when_connected() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();

        let msg = TelemetryMessage::heartbeat(1000);
        assert!(server.send_telemetry(msg).is_ok());
        assert_eq!(server.tx_queue_len(), 1);
    }

    #[test]
    fn pop_telemetry_returns_messages_in_order() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();

        let msg1 = TelemetryMessage::new(TelemetryType::Motor, 100);
        let msg2 = TelemetryMessage::new(TelemetryType::Imu, 200);

        server.send_telemetry(msg1).unwrap();
        server.send_telemetry(msg2).unwrap();

        let popped1 = server.pop_telemetry().unwrap();
        assert_eq!(popped1.msg_type, TelemetryType::Motor);
        assert_eq!(popped1.timestamp_ms, 100);

        let popped2 = server.pop_telemetry().unwrap();
        assert_eq!(popped2.msg_type, TelemetryType::Imu);

        assert!(server.pop_telemetry().is_none());
    }

    #[test]
    fn telemetry_buffer_full_error() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();

        // Fill the queue (capacity is 8)
        for i in 0..8 {
            let msg = TelemetryMessage::heartbeat(i * 100);
            assert!(server.send_telemetry(msg).is_ok());
        }

        // Next should fail
        let msg = TelemetryMessage::heartbeat(900);
        assert_eq!(server.send_telemetry(msg), Err(CommsError::BufferFull));
    }

    // === Command Tests ===

    #[test]
    fn receive_command_queues_it() {
        let mut server = make_server();
        let cmd = RemoteCommand::Ping;
        assert!(server.receive_command(cmd).is_ok());
        assert_eq!(server.rx_queue_len(), 1);
        assert_eq!(server.stats().rx_count, 1);
    }

    #[test]
    fn pop_command_returns_in_order() {
        let mut server = make_server();

        server.receive_command(RemoteCommand::Ping).unwrap();
        server.receive_command(RemoteCommand::EmergencyStop).unwrap();

        assert_eq!(server.pop_command(), Some(RemoteCommand::Ping));
        assert_eq!(server.pop_command(), Some(RemoteCommand::EmergencyStop));
        assert_eq!(server.pop_command(), None);
    }

    #[test]
    fn command_buffer_full_error() {
        let mut server = make_server();

        // Fill the queue (capacity is 8)
        for _ in 0..8 {
            assert!(server.receive_command(RemoteCommand::Ping).is_ok());
        }

        // Next should fail
        assert_eq!(
            server.receive_command(RemoteCommand::Ping),
            Err(CommsError::BufferFull)
        );
    }

    // === Update / Heartbeat Tests ===

    #[test]
    fn update_queues_heartbeat_when_connected() {
        let mut config = CommsConfig::default();
        config.heartbeat_interval_ms = 100;
        let mut server = CommsServer::new(config);

        server.start_connect();
        server.on_connected();

        // First update at t=0
        server.update(0);
        assert_eq!(server.tx_queue_len(), 0); // No heartbeat yet (just connected)

        // Update at t=100 (heartbeat interval)
        server.update(100);
        assert_eq!(server.tx_queue_len(), 1);

        let hb = server.pop_telemetry().unwrap();
        assert_eq!(hb.msg_type, TelemetryType::Heartbeat);
    }

    #[test]
    fn update_does_nothing_when_disconnected() {
        let mut server = make_server();
        server.update(1000);
        assert_eq!(server.tx_queue_len(), 0);
    }

    // === Statistics Tests ===

    #[test]
    fn stats_start_at_zero() {
        let server = make_server();
        let stats = server.stats();
        assert_eq!(stats.tx_count, 0);
        assert_eq!(stats.rx_count, 0);
        assert_eq!(stats.error_count, 0);
    }

    #[test]
    fn on_message_sent_increments_tx_count() {
        let mut server = make_server();
        server.on_message_sent();
        server.on_message_sent();
        assert_eq!(server.stats().tx_count, 2);
    }

    #[test]
    fn reconnect_count_increments() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();
        // First connect doesn't count as reconnect
        assert_eq!(server.stats().reconnect_count, 0);

        // Simulate sending something (indicates was connected)
        server.on_message_sent();

        // Disconnect and reconnect
        server.on_disconnected();
        server.start_connect();
        server.on_connected();
        assert_eq!(server.stats().reconnect_count, 1);
    }

    // === Config Tests ===

    #[test]
    fn config_ssid_accessible() {
        let config = make_config("TestNetwork");
        let server = CommsServer::new(config);
        assert_eq!(server.ssid(), "TestNetwork");
    }

    #[test]
    fn default_config_reasonable() {
        let config = CommsConfig::default();
        assert_eq!(config.telemetry_rate_hz, 10);
        assert_eq!(config.heartbeat_interval_ms, 1000);
        assert!(config.wifi.use_dhcp);
    }

    // === Queue Management Tests ===

    #[test]
    fn clear_queues_empties_both() {
        let mut server = make_server();
        server.start_connect();
        server.on_connected();

        server.send_telemetry(TelemetryMessage::heartbeat(0)).unwrap();
        server.receive_command(RemoteCommand::Ping).unwrap();

        assert_eq!(server.tx_queue_len(), 1);
        assert_eq!(server.rx_queue_len(), 1);

        server.clear_queues();

        assert_eq!(server.tx_queue_len(), 0);
        assert_eq!(server.rx_queue_len(), 0);
    }

    // === Message Type Tests ===

    #[test]
    fn telemetry_message_new_creates_empty_payload() {
        let msg = TelemetryMessage::new(TelemetryType::Motor, 500);
        assert_eq!(msg.msg_type, TelemetryType::Motor);
        assert_eq!(msg.timestamp_ms, 500);
        assert!(msg.payload.is_empty());
    }

    #[test]
    fn heartbeat_message_has_payload() {
        let hb = TelemetryMessage::heartbeat(1234);
        assert_eq!(hb.msg_type, TelemetryType::Heartbeat);
        assert_eq!(hb.timestamp_ms, 1234);
        assert_eq!(&hb.payload[..], b"OK");
    }

    // === Remote Command Tests ===

    #[test]
    fn remote_commands_are_comparable() {
        let cmd1 = RemoteCommand::SetVelocity { left: 100, right: 100 };
        let cmd2 = RemoteCommand::SetVelocity { left: 100, right: 100 };
        let cmd3 = RemoteCommand::EmergencyStop;

        assert_eq!(cmd1, cmd2);
        assert_ne!(cmd1, cmd3);
    }

    #[test]
    fn all_telemetry_types_exist() {
        let types = [
            TelemetryType::Motor,
            TelemetryType::Imu,
            TelemetryType::Pose,
            TelemetryType::System,
            TelemetryType::Heartbeat,
        ];
        assert_eq!(types.len(), 5);
    }
}
