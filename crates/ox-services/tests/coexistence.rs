//! Coexistence Integration Tests
//!
//! Tests WiFi + BLE + RC working together:
//! - Resource sharing between systems
//! - Priority handling
//! - No interference between subsystems
//! - Failsafe behavior when one system fails

use ox_services::comms::{
    CommsServer, ConnectionState, RemoteCommand, TelemetryMessage, TelemetryType,
};
use ox_services::rc::{ChannelMap, FailsafeConfig, RcServer, SbusDecoder};
use ox_test::{
    make_sbus_center_frame, make_sbus_control_frame, CommsTestHarness, RcTestHarness,
    TimingTestHarness,
};
use std::time::Instant;

/// Test RC and Comms can run simultaneously without interference
#[test]
fn rc_and_comms_concurrent_operation() {
    let mut rc_decoder = SbusDecoder::new();
    let mut rc_server = RcServer::default();
    let mut rc_harness = RcTestHarness::new();

    let mut comms = CommsServer::default();
    let mut comms_harness = CommsTestHarness::new();

    // Start comms connection
    comms.start_connect();
    comms_harness.record_state("Connecting");
    comms.on_connected();
    comms_harness.record_state("Connected");

    // Process RC frames while sending telemetry
    for i in 0..100 {
        // RC: Decode frame and process
        let frame = make_sbus_control_frame(0.5, 0.1);
        for byte in frame {
            if let Some(ch) = rc_decoder.decode(byte) {
                rc_server.process(&ch);
                rc_harness.record_valid_frame();
            }
        }

        // Comms: Update and send telemetry
        comms.update(i * 10);

        // Drain queue to make room (simulates network send)
        while let Some(_msg) = comms.pop_telemetry() {
            comms.on_message_sent();
        }

        if i % 10 == 0 {
            let msg = TelemetryMessage::new(TelemetryType::Motor, i);
            if comms.send_telemetry(msg).is_ok() {
                comms_harness.record_telemetry();
            }
        }

        // Verify RC still works
        assert!(!rc_server.is_failsafe());
        assert!((rc_server.throttle() - 0.5).abs() < 0.1);

        // Verify comms still connected
        assert!(comms.is_connected());
    }

    // Final assertions
    assert_eq!(rc_harness.valid_frame_count, 100);
    assert!(comms_harness.telemetry_sent >= 10);
}

/// Test RC failsafe doesn't affect comms
#[test]
fn rc_failsafe_independent_of_comms() {
    let mut config = FailsafeConfig::default();
    config.timeout_ms = 100;
    let mut rc_server = RcServer::new(ChannelMap::default(), config);

    let mut comms = CommsServer::default();
    comms.start_connect();
    comms.on_connected();

    // Get initial RC signal
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_center_frame();
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }
    assert!(!rc_server.is_failsafe());

    // Trigger RC failsafe
    rc_server.update(150);
    assert!(rc_server.is_failsafe());

    // Comms should still be connected
    assert!(comms.is_connected());
    let msg = TelemetryMessage::heartbeat(1000);
    assert!(comms.send_telemetry(msg).is_ok());
}

/// Test comms failure doesn't affect RC
#[test]
fn comms_failure_independent_of_rc() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();

    // Setup both systems
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_center_frame();
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }

    comms.start_connect();
    comms.on_connected();

    assert!(!rc_server.is_failsafe());
    assert!(comms.is_connected());

    // Comms fails
    comms.on_error();
    assert_eq!(comms.state(), ConnectionState::Error);

    // RC should still work
    assert!(!rc_server.is_failsafe());
    assert!(rc_server.throttle().abs() < 0.1);

    // Process more RC frames
    let frame = make_sbus_control_frame(0.8, 0.2);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }

    // RC should have updated values despite comms failure
    assert!(rc_server.throttle() > 0.7);
}

/// Test combined timing when all systems active
#[test]
fn combined_systems_timing() {
    let mut harness = TimingTestHarness::new(500); // 500us budget for combined ops

    let frame = make_sbus_center_frame();

    for i in 0..1000 {
        let mut decoder = SbusDecoder::new();
        let mut rc_server = RcServer::default();
        let mut comms = CommsServer::default();
        comms.start_connect();
        comms.on_connected();

        let start = Instant::now();

        // RC operations
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                rc_server.process(&ch);
            }
        }
        let _throttle = rc_server.throttle();
        let _steering = rc_server.steering();
        rc_server.update(10);

        // Comms operations
        comms.update(i);
        if i % 10 == 0 {
            let msg = TelemetryMessage::new(TelemetryType::Motor, i);
            let _ = comms.send_telemetry(msg);
        }
        while let Some(_msg) = comms.pop_telemetry() {
            comms.on_message_sent();
        }

        let elapsed = start.elapsed().as_micros() as u64;
        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Combined timing {} μs exceeds 500 μs",
        harness.max()
    );

    println!(
        "Combined RC+Comms: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test command routing from comms to motor (via RC override)
#[test]
fn remote_command_coexists_with_rc() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();

    // Setup RC
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_control_frame(0.5, 0.0);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }

    // Setup comms
    comms.start_connect();
    comms.on_connected();

    // RC provides base control
    let rc_throttle = rc_server.throttle();
    assert!((rc_throttle - 0.5).abs() < 0.1);

    // Remote command comes in
    let cmd = RemoteCommand::SetVelocity {
        left: 800,
        right: 800,
    };
    comms.receive_command(cmd).unwrap();

    // Pop and verify command
    let received = comms.pop_command().unwrap();
    assert_eq!(
        received,
        RemoteCommand::SetVelocity {
            left: 800,
            right: 800
        }
    );

    // RC should still report its values (not affected by remote command)
    assert!((rc_server.throttle() - 0.5).abs() < 0.1);
}

/// Test emergency stop through comms while RC active
#[test]
fn emergency_stop_via_comms_with_active_rc() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();

    // RC has high throttle
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_control_frame(1.0, 0.0);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }
    assert!(rc_server.throttle() > 0.9);

    // Comms sends emergency stop
    comms.start_connect();
    comms.on_connected();
    comms.receive_command(RemoteCommand::EmergencyStop).unwrap();

    let cmd = comms.pop_command().unwrap();
    assert_eq!(cmd, RemoteCommand::EmergencyStop);

    // Note: In real implementation, emergency stop would override RC
    // Here we just verify the command is received correctly
    // The motor server would handle the priority
}

/// Test telemetry includes RC status
#[test]
fn telemetry_reports_rc_status() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();

    comms.start_connect();
    comms.on_connected();

    // RC in failsafe
    assert!(rc_server.is_failsafe()); // Starts in failsafe

    // Create telemetry with RC info
    let mut msg = TelemetryMessage::new(TelemetryType::System, 1000);
    let failsafe_byte: u8 = if rc_server.is_failsafe() { 1 } else { 0 };
    msg.payload.push(failsafe_byte).unwrap();

    comms.send_telemetry(msg).unwrap();

    let sent = comms.pop_telemetry().unwrap();
    assert_eq!(sent.payload[0], 1); // Failsafe = true

    // Get RC signal
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_center_frame();
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }
    assert!(!rc_server.is_failsafe());

    // New telemetry reflects updated status
    let mut msg2 = TelemetryMessage::new(TelemetryType::System, 2000);
    let failsafe_byte: u8 = if rc_server.is_failsafe() { 1 } else { 0 };
    msg2.payload.push(failsafe_byte).unwrap();

    comms.send_telemetry(msg2).unwrap();
    let sent2 = comms.pop_telemetry().unwrap();
    assert_eq!(sent2.payload[0], 0); // Failsafe = false
}

/// Test reconnection doesn't interrupt RC
#[test]
fn comms_reconnect_preserves_rc() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();
    let mut rc_harness = RcTestHarness::new();

    // Setup both
    let mut decoder = SbusDecoder::new();
    let frame = make_sbus_control_frame(0.6, -0.3);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
            rc_harness.record_valid_frame();
        }
    }

    comms.start_connect();
    comms.on_connected();
    comms.on_message_sent();

    // Comms disconnects and reconnects
    comms.on_disconnected();
    assert_eq!(comms.state(), ConnectionState::Reconnecting);

    // RC still working during disconnect
    let frame2 = make_sbus_control_frame(0.7, 0.1);
    for byte in frame2 {
        if let Some(ch) = decoder.decode(byte) {
            rc_server.process(&ch);
            rc_harness.record_valid_frame();
        }
    }
    assert!(!rc_server.is_failsafe());
    assert!(rc_server.throttle() > 0.6);

    // Reconnect
    comms.start_connect();
    comms.on_connected();

    // Both systems operational
    assert!(comms.is_connected());
    assert!(!rc_server.is_failsafe());
    assert_eq!(rc_harness.valid_frame_count, 2);
}

/// Test multiple RC frame types work with comms
#[test]
fn multiple_rc_protocols_with_comms() {
    use ox_services::rc::CrsfDecoder;
    use ox_test::make_crsf_center_frame;

    let mut sbus_decoder = SbusDecoder::new();
    let mut crsf_decoder = CrsfDecoder::new();
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();

    comms.start_connect();
    comms.on_connected();

    // Process SBUS
    let sbus_frame = make_sbus_center_frame();
    for byte in sbus_frame {
        if let Some(ch) = sbus_decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }
    assert!(!rc_server.is_failsafe());

    // Comms update
    comms.update(100);

    // Process CRSF (simulating protocol switch)
    let crsf_frame = make_crsf_center_frame();
    for byte in crsf_frame {
        if let Some(ch) = crsf_decoder.decode(byte) {
            rc_server.process(&ch);
        }
    }
    assert!(!rc_server.is_failsafe());

    // Both protocols work, comms unaffected
    assert!(comms.is_connected());
}

/// Test system stats collection from multiple sources
#[test]
fn combined_system_statistics() {
    let mut rc_server = RcServer::default();
    let mut comms = CommsServer::default();
    let mut rc_harness = RcTestHarness::new();
    let mut comms_harness = CommsTestHarness::new();

    comms.start_connect();
    comms_harness.record_state("Connecting");
    comms.on_connected();
    comms_harness.record_state("Connected");

    // Process frames and collect stats
    let mut decoder = SbusDecoder::new();
    for i in 0..50 {
        let frame = make_sbus_center_frame();
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                rc_server.process(&ch);
                rc_harness.record_valid_frame();
            }
        }

        // Drain queue to simulate network send
        while let Some(_msg) = comms.pop_telemetry() {
            comms.on_message_sent();
        }

        if i % 5 == 0 {
            let msg = TelemetryMessage::heartbeat(i * 100);
            if comms.send_telemetry(msg).is_ok() {
                comms_harness.record_telemetry();
            }
        }
    }

    // Verify stats
    assert_eq!(rc_harness.valid_frame_count, 50);
    assert_eq!(rc_harness.failsafe_count, 0);
    assert!(comms_harness.telemetry_sent >= 10);
    assert!(comms_harness.verify_transitions(&["Connecting", "Connected"]));
}
