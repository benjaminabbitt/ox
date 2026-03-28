//! RC System Integration Tests
//!
//! Tests the full RC pipeline:
//! - SBUS/CRSF frame decoding
//! - RcServer channel processing
//! - Motor command generation (differential drive)
//! - Failsafe behavior

use ox_services::rc::{
    ChannelMap, CrsfDecoder, FailsafeConfig, FailsafeMode, RcServer, SbusDecoder,
};
use ox_test::{
    make_crsf_center_frame, make_crsf_channels_frame, make_crsf_link_stats_frame,
    make_sbus_center_frame, make_sbus_control_frame, make_sbus_frame, RcTestHarness,
};

/// Test full SBUS decode → RcServer pipeline
#[test]
fn sbus_full_pipeline_center_sticks() {
    let mut decoder = SbusDecoder::new();
    let mut server = RcServer::default();
    let mut harness = RcTestHarness::new();

    // Feed center frame
    let frame = make_sbus_center_frame();
    let mut channels = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            channels = Some(ch);
        }
    }

    let ch = channels.expect("Should decode frame");
    harness.record_valid_frame();

    // Process through server
    server.process(&ch);
    assert!(!server.is_failsafe());

    // With center sticks, throttle and steering should be ~0
    let throttle = server.throttle();
    let steering = server.steering();
    assert!(
        throttle.abs() < 0.05,
        "Center throttle should be ~0, got {}",
        throttle
    );
    assert!(
        steering.abs() < 0.05,
        "Center steering should be ~0, got {}",
        steering
    );

    // Generate motor command
    let left = ((throttle + steering) * 1000.0) as i16;
    let right = ((throttle - steering) * 1000.0) as i16;
    harness.record_command(left, right);

    // Verify differential drive
    assert!(harness.verify_differential_drive(throttle, steering));
}

/// Test SBUS control input → motor output
#[test]
fn sbus_control_to_motor_command() {
    let mut decoder = SbusDecoder::new();
    let mut server = RcServer::default();
    let mut harness = RcTestHarness::new();

    // Forward at half throttle, slight right turn
    let frame = make_sbus_control_frame(0.5, 0.2);
    let mut channels = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            channels = Some(ch);
        }
    }

    let ch = channels.expect("Should decode frame");
    server.process(&ch);
    harness.record_valid_frame();

    let throttle = server.throttle();
    let steering = server.steering();

    // Generate motor command
    let left = ((throttle + steering) * 1000.0) as i16;
    let right = ((throttle - steering) * 1000.0) as i16;
    harness.record_command(left, right);

    // Right turn means left > right
    assert!(left > right, "Left ({}) should be > right ({})", left, right);

    // Verify expected values (within tolerance)
    assert!(harness.verify_differential_drive(throttle, steering));
}

/// Test SBUS failsafe detection and response
#[test]
fn sbus_failsafe_triggers_safe_behavior() {
    let mut decoder = SbusDecoder::new();

    let mut config = FailsafeConfig::default();
    config.mode = FailsafeMode::Neutral;
    config.timeout_ms = 100;
    let mut server = RcServer::new(ChannelMap::default(), config);

    let mut harness = RcTestHarness::new();

    // First, establish valid signal
    let frame = make_sbus_control_frame(0.8, 0.0); // High throttle
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
            harness.record_valid_frame();
        }
    }

    assert!(!server.is_failsafe());
    assert!(server.throttle() > 0.7);

    // Simulate signal loss (no frames for timeout period)
    server.update(150); // 150ms > 100ms timeout
    harness.record_failsafe();

    assert!(server.is_failsafe());
    assert_eq!(harness.failsafe_count, 1);

    // In Neutral mode, throttle should be 0
    assert!(
        server.throttle().abs() < 0.01,
        "Failsafe should return neutral, got {}",
        server.throttle()
    );
}

/// Test SBUS hold mode preserves last values
#[test]
fn sbus_failsafe_hold_mode() {
    let mut decoder = SbusDecoder::new();

    let mut config = FailsafeConfig::default();
    config.mode = FailsafeMode::Hold;
    config.timeout_ms = 100;
    let mut server = RcServer::new(ChannelMap::default(), config);

    // Set throttle to 0.5
    let frame = make_sbus_control_frame(0.5, 0.1);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
        }
    }

    let last_throttle = server.throttle();
    let last_steering = server.steering();

    // Trigger failsafe
    server.update(150);
    assert!(server.is_failsafe());

    // Values should be held
    assert!(
        (server.throttle() - last_throttle).abs() < 0.01,
        "Hold mode should preserve throttle"
    );
    assert!(
        (server.steering() - last_steering).abs() < 0.01,
        "Hold mode should preserve steering"
    );
}

/// Test CRSF full pipeline
#[test]
fn crsf_full_pipeline() {
    let mut decoder = CrsfDecoder::new();
    let mut server = RcServer::default();
    let mut harness = RcTestHarness::new();

    // Feed center frame
    let frame = make_crsf_center_frame();
    let mut channels = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            channels = Some(ch);
        }
    }

    let ch = channels.expect("Should decode CRSF frame");
    harness.record_valid_frame();
    server.process(&ch);

    assert!(!server.is_failsafe());
    assert!(server.throttle().abs() < 0.05);
    assert!(server.steering().abs() < 0.05);
}

/// Test CRSF with link stats
#[test]
fn crsf_with_link_statistics() {
    let mut decoder = CrsfDecoder::new();

    // First send link stats
    let link_frame = make_crsf_link_stats_frame(-80, 95, 10);
    for byte in link_frame {
        let _ = decoder.decode(byte);
    }

    // Link quality should be updated
    assert_eq!(decoder.link_quality(), 95);

    // Then send channels
    let ch_frame = make_crsf_center_frame();
    let mut channels = None;
    for byte in ch_frame {
        if let Some(ch) = decoder.decode(byte) {
            channels = Some(ch);
        }
    }

    let ch = channels.unwrap();
    assert_eq!(ch.link_quality, Some(95));
}

/// Test CRSF control inputs
#[test]
fn crsf_control_inputs() {
    let mut decoder = CrsfDecoder::new();
    let mut server = RcServer::default();

    // Create channels with specific values
    let mut channels = [992u16; 16]; // Center values
    channels[2] = 1811; // Max throttle (channel 3)
    channels[0] = 172; // Min steering (channel 1)

    let frame = make_crsf_channels_frame(&channels);
    let mut decoded = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            decoded = Some(ch);
        }
    }

    let ch = decoded.expect("Should decode frame");
    server.process(&ch);

    // Max throttle
    assert!(server.throttle() > 0.9, "Max throttle expected");
    // Min steering (hard left)
    assert!(server.steering() < -0.9, "Min steering expected");
}

/// Test multiple frame decode sequence
#[test]
fn multiple_sbus_frames_sequence() {
    let mut decoder = SbusDecoder::new();
    let mut server = RcServer::default();
    let mut harness = RcTestHarness::new();

    let throttle_sequence = [0.0, 0.25, 0.5, 0.75, 1.0, 0.75, 0.5, 0.25, 0.0];

    for &throttle in &throttle_sequence {
        let frame = make_sbus_control_frame(throttle, 0.0);
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                server.process(&ch);
                harness.record_valid_frame();

                let actual = server.throttle();
                assert!(
                    (actual - throttle).abs() < 0.05,
                    "Expected throttle ~{}, got {}",
                    throttle,
                    actual
                );
            }
        }
    }

    assert_eq!(harness.valid_frame_count, throttle_sequence.len() as u32);
}

/// Test frame corruption recovery
#[test]
fn sbus_corruption_recovery() {
    let mut decoder = SbusDecoder::new();

    // Send corrupt bytes (not starting with 0x0F)
    for _ in 0..10 {
        assert!(decoder.decode(0xFF).is_none());
        assert!(decoder.decode(0x00).is_none());
        assert!(decoder.decode(0xAB).is_none());
    }

    // Now send valid frame - should recover
    let frame = make_sbus_center_frame();
    let mut result = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            result = Some(ch);
        }
    }

    assert!(result.is_some(), "Should decode valid frame after corruption");
    assert!(!result.unwrap().failsafe);
}

/// Test arm switch behavior
#[test]
fn arm_switch_behavior() {
    let mut decoder = SbusDecoder::new();
    let mut server = RcServer::default();

    // Create frame with aux2 (ch6) low
    let mut channels = [992u16; 16];
    channels[5] = 172; // Aux2 low = disarmed

    let frame = make_sbus_frame(&channels, false, false);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
        }
    }

    assert!(!server.is_armed(), "Should be disarmed");

    // Set aux2 high
    channels[5] = 1811; // Aux2 high = armed
    let frame = make_sbus_frame(&channels, false, false);
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
        }
    }

    assert!(server.is_armed(), "Should be armed");
}

/// Test channel mapping
#[test]
fn custom_channel_mapping() {
    let mut map = ChannelMap::default();
    map.throttle = 1; // Use ch2 for throttle
    map.steering = 3; // Use ch4 for steering

    let server = RcServer::new(map, FailsafeConfig::default());
    assert_eq!(server.channel_map().throttle, 1);
    assert_eq!(server.channel_map().steering, 3);
}

/// Test harness tracks commands correctly
#[test]
fn harness_records_motor_commands() {
    let mut harness = RcTestHarness::new();

    // Simulate a sequence of motor commands
    harness.record_command(500, 500); // Forward
    harness.record_command(700, 300); // Right turn
    harness.record_command(0, 0); // Stop

    assert_eq!(harness.motor_commands.len(), 3);
    assert_eq!(harness.last_command(), Some((0, 0)));
}

/// Test failsafe recovery
#[test]
fn failsafe_recovery() {
    let mut decoder = SbusDecoder::new();
    let mut config = FailsafeConfig::default();
    config.timeout_ms = 100;
    let mut server = RcServer::new(ChannelMap::default(), config);

    // Establish signal
    let frame = make_sbus_center_frame();
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
        }
    }
    assert!(!server.is_failsafe());

    // Trigger failsafe
    server.update(150);
    assert!(server.is_failsafe());

    // Recovery - new valid frame
    let frame = make_sbus_center_frame();
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            server.process(&ch);
        }
    }

    assert!(!server.is_failsafe(), "Should recover from failsafe");
}
