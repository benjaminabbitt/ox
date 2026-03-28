//! Communications System Integration Tests
//!
//! Tests the CommsServer state machine and telemetry pipeline:
//! - Connection state transitions
//! - Telemetry queuing and retrieval
//! - Command processing
//! - Heartbeat generation
//! - Error handling and recovery

use ox_services::comms::{
    CommsConfig, CommsError, CommsServer, ConnectionState, RemoteCommand, TelemetryMessage,
    TelemetryType,
};
use ox_test::CommsTestHarness;

fn make_server_with_heartbeat(interval_ms: u32) -> CommsServer {
    let mut config = CommsConfig::default();
    config.heartbeat_interval_ms = interval_ms;
    CommsServer::new(config)
}

/// Test complete connection lifecycle
#[test]
fn connection_lifecycle() {
    let mut server = CommsServer::default();
    let mut harness = CommsTestHarness::new();

    // Initial state
    harness.record_state("Disconnected");
    assert_eq!(server.state(), ConnectionState::Disconnected);

    // Start connecting
    server.start_connect();
    harness.record_state("Connecting");
    assert_eq!(server.state(), ConnectionState::Connecting);

    // Connected
    server.on_connected();
    harness.record_state("Connected");
    assert_eq!(server.state(), ConnectionState::Connected);
    assert!(server.is_connected());

    // Verify state transitions
    assert!(harness.verify_transitions(&["Disconnected", "Connecting", "Connected"]));
}

/// Test reconnection flow
#[test]
fn reconnection_flow() {
    let mut server = CommsServer::default();
    let mut harness = CommsTestHarness::new();

    // Connect
    server.start_connect();
    server.on_connected();
    server.on_message_sent(); // Mark as having sent data

    // Lose connection
    server.on_disconnected();
    harness.record_state("Reconnecting");
    assert_eq!(server.state(), ConnectionState::Reconnecting);

    // Reconnect
    server.start_connect();
    harness.record_state("Connecting");
    server.on_connected();
    harness.record_state("Connected");

    assert_eq!(server.stats().reconnect_count, 1);
    assert!(harness.verify_transitions(&["Reconnecting", "Connecting", "Connected"]));
}

/// Test error state handling
#[test]
fn error_state_handling() {
    let mut server = CommsServer::default();

    server.on_error();
    assert_eq!(server.state(), ConnectionState::Error);
    assert_eq!(server.stats().error_count, 1);

    // Multiple errors accumulate
    server.on_error();
    assert_eq!(server.stats().error_count, 2);
}

/// Test telemetry send/receive flow
#[test]
fn telemetry_flow() {
    let mut server = CommsServer::default();
    let mut harness = CommsTestHarness::new();

    // Must connect first
    server.start_connect();
    server.on_connected();

    // Send telemetry
    let msg1 = TelemetryMessage::new(TelemetryType::Motor, 100);
    let msg2 = TelemetryMessage::new(TelemetryType::Imu, 200);
    let msg3 = TelemetryMessage::new(TelemetryType::Pose, 300);

    assert!(server.send_telemetry(msg1).is_ok());
    harness.record_telemetry();
    assert!(server.send_telemetry(msg2).is_ok());
    harness.record_telemetry();
    assert!(server.send_telemetry(msg3).is_ok());
    harness.record_telemetry();

    assert_eq!(server.tx_queue_len(), 3);
    assert_eq!(harness.telemetry_sent, 3);

    // Retrieve in order
    let popped = server.pop_telemetry().unwrap();
    assert_eq!(popped.msg_type, TelemetryType::Motor);
    assert_eq!(popped.timestamp_ms, 100);

    let popped = server.pop_telemetry().unwrap();
    assert_eq!(popped.msg_type, TelemetryType::Imu);

    let popped = server.pop_telemetry().unwrap();
    assert_eq!(popped.msg_type, TelemetryType::Pose);

    assert!(server.pop_telemetry().is_none());
}

/// Test command queuing
#[test]
fn command_flow() {
    let mut server = CommsServer::default();
    let mut harness = CommsTestHarness::new();

    // Queue commands
    let cmds = [
        RemoteCommand::Ping,
        RemoteCommand::SetVelocity {
            left: 100,
            right: 100,
        },
        RemoteCommand::EmergencyStop,
    ];

    for cmd in &cmds {
        assert!(server.receive_command(*cmd).is_ok());
        harness.record_command();
    }

    assert_eq!(harness.commands_received, 3);
    assert_eq!(server.rx_queue_len(), 3);

    // Process in order
    assert_eq!(server.pop_command(), Some(RemoteCommand::Ping));
    assert_eq!(
        server.pop_command(),
        Some(RemoteCommand::SetVelocity {
            left: 100,
            right: 100
        })
    );
    assert_eq!(server.pop_command(), Some(RemoteCommand::EmergencyStop));
    assert!(server.pop_command().is_none());
}

/// Test heartbeat generation
#[test]
fn heartbeat_generation() {
    let mut server = make_server_with_heartbeat(100);

    server.start_connect();
    server.on_connected();

    // Initial update - no heartbeat yet
    server.update(0);
    assert_eq!(server.tx_queue_len(), 0);

    // Update at heartbeat interval
    server.update(100);
    assert_eq!(server.tx_queue_len(), 1);

    let hb = server.pop_telemetry().unwrap();
    assert_eq!(hb.msg_type, TelemetryType::Heartbeat);
    assert_eq!(&hb.payload[..], b"OK");

    // Multiple heartbeats over time
    server.update(200);
    assert_eq!(server.tx_queue_len(), 1);
    server.update(300);
    assert_eq!(server.tx_queue_len(), 2);
}

/// Test telemetry requires connection
#[test]
fn telemetry_requires_connection() {
    let mut server = CommsServer::default();

    let msg = TelemetryMessage::heartbeat(0);
    assert_eq!(server.send_telemetry(msg), Err(CommsError::NotConnected));

    // Now connect
    server.start_connect();
    server.on_connected();

    let msg = TelemetryMessage::heartbeat(0);
    assert!(server.send_telemetry(msg).is_ok());
}

/// Test queue capacity limits
#[test]
fn queue_capacity_limits() {
    let mut server = CommsServer::default();
    server.start_connect();
    server.on_connected();

    // Fill telemetry queue (capacity 8)
    for i in 0..8 {
        let msg = TelemetryMessage::new(TelemetryType::System, i * 100);
        assert!(server.send_telemetry(msg).is_ok());
    }

    // Next should fail
    let msg = TelemetryMessage::new(TelemetryType::System, 800);
    assert_eq!(server.send_telemetry(msg), Err(CommsError::BufferFull));

    // Fill command queue (capacity 8)
    for _ in 0..8 {
        assert!(server.receive_command(RemoteCommand::Ping).is_ok());
    }

    // Next should fail
    assert_eq!(
        server.receive_command(RemoteCommand::Ping),
        Err(CommsError::BufferFull)
    );
}

/// Test queue clear
#[test]
fn queue_clear() {
    let mut server = CommsServer::default();
    server.start_connect();
    server.on_connected();

    server
        .send_telemetry(TelemetryMessage::heartbeat(0))
        .unwrap();
    server.receive_command(RemoteCommand::Ping).unwrap();

    assert_eq!(server.tx_queue_len(), 1);
    assert_eq!(server.rx_queue_len(), 1);

    server.clear_queues();

    assert_eq!(server.tx_queue_len(), 0);
    assert_eq!(server.rx_queue_len(), 0);
}

/// Test statistics tracking
#[test]
fn statistics_tracking() {
    let mut server = CommsServer::default();

    // Initial stats
    let stats = server.stats();
    assert_eq!(stats.tx_count, 0);
    assert_eq!(stats.rx_count, 0);
    assert_eq!(stats.error_count, 0);
    assert_eq!(stats.reconnect_count, 0);

    // Send messages
    server.start_connect();
    server.on_connected();

    server.on_message_sent();
    server.on_message_sent();
    server.receive_command(RemoteCommand::Ping).unwrap();

    let stats = server.stats();
    assert_eq!(stats.tx_count, 2);
    assert_eq!(stats.rx_count, 1);
}

/// Test WiFi config
#[test]
fn wifi_config() {
    let mut config = CommsConfig::default();
    config.wifi.ssid = "TestNetwork".try_into().unwrap();
    config.wifi.password = "TestPassword".try_into().unwrap();
    config.wifi.use_dhcp = true;

    let server = CommsServer::new(config);
    assert_eq!(server.ssid(), "TestNetwork");
}

/// Test motor velocity command handling
#[test]
fn motor_velocity_command() {
    let mut server = CommsServer::default();
    let mut harness = CommsTestHarness::new();

    let cmd = RemoteCommand::SetVelocity {
        left: 500,
        right: 500,
    };
    server.receive_command(cmd).unwrap();
    harness.record_command();

    if let Some(RemoteCommand::SetVelocity { left, right }) = server.pop_command() {
        assert_eq!(left, 500);
        assert_eq!(right, 500);
    } else {
        panic!("Expected velocity command");
    }
}

/// Test emergency stop command
#[test]
fn emergency_stop_command() {
    let mut server = CommsServer::default();

    server.receive_command(RemoteCommand::EmergencyStop).unwrap();

    let cmd = server.pop_command().unwrap();
    assert_eq!(cmd, RemoteCommand::EmergencyStop);
}

/// Test disconnected state doesn't queue heartbeats
#[test]
fn disconnected_no_heartbeat() {
    let mut server = make_server_with_heartbeat(100);

    // Update while disconnected
    server.update(0);
    server.update(100);
    server.update(200);

    assert_eq!(server.tx_queue_len(), 0);
}

/// Test connection time tracking
#[test]
fn connection_time_tracking() {
    let mut server = CommsServer::default();

    assert_eq!(server.stats().connected_time_ms, 0);

    server.start_connect();
    server.on_connected();

    // Each update increments connected time
    server.update(0);
    server.update(1);
    server.update(2);

    assert!(server.stats().connected_time_ms >= 3);
}

/// Test harness state transition tracking
#[test]
fn harness_state_transitions() {
    let mut harness = CommsTestHarness::new();

    harness.record_state("Disconnected");
    harness.record_state("Connecting");
    harness.record_state("Connected");
    harness.record_state("Reconnecting");
    harness.record_state("Connected");

    assert!(harness.verify_transitions(&[
        "Disconnected",
        "Connecting",
        "Connected",
        "Reconnecting",
        "Connected"
    ]));

    // Wrong sequence fails
    assert!(!harness.verify_transitions(&["Disconnected", "Connected"]));
}

/// Test telemetry message types
#[test]
fn all_telemetry_types() {
    let mut server = CommsServer::default();
    server.start_connect();
    server.on_connected();

    let types = [
        TelemetryType::Motor,
        TelemetryType::Imu,
        TelemetryType::Pose,
        TelemetryType::System,
        TelemetryType::Heartbeat,
    ];

    for (i, msg_type) in types.iter().enumerate() {
        let msg = TelemetryMessage::new(*msg_type, i as u32 * 100);
        assert!(server.send_telemetry(msg).is_ok());
    }

    assert_eq!(server.tx_queue_len(), 5);
}
