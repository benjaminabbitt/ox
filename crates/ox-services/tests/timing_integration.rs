//! Timing Integration Tests
//!
//! Tests timing requirements and constraints:
//! - Control loop timing budget
//! - RC frame processing time
//! - Telemetry latency
//! - Failsafe timeout accuracy

use ox_services::comms::{CommsConfig, CommsServer, TelemetryMessage, TelemetryType};
use ox_services::rc::{CrsfDecoder, RcServer, SbusDecoder};
use ox_test::{make_sbus_center_frame, make_crsf_center_frame, TimingTestHarness};
use std::time::Instant;

/// Timing budget for control loop (microseconds)
const CONTROL_LOOP_BUDGET_US: u64 = 1000; // 1ms target

/// Target success rate for timing tests
const TARGET_SUCCESS_RATE: f32 = 99.0; // 99% of operations within budget

/// Test SBUS decode timing
#[test]
fn sbus_decode_timing() {
    let mut harness = TimingTestHarness::new(100); // 100us budget for decode
    let frame = make_sbus_center_frame();

    for _ in 0..1000 {
        let mut decoder = SbusDecoder::new();

        let start = Instant::now();
        for byte in frame {
            let _ = decoder.decode(byte);
        }
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "SBUS decode max {} μs exceeds 100 μs budget",
        harness.max()
    );
    assert!(
        harness.success_rate() >= TARGET_SUCCESS_RATE,
        "Success rate {} < {}%",
        harness.success_rate(),
        TARGET_SUCCESS_RATE
    );

    println!(
        "SBUS decode: min={} μs, max={} μs, avg={} μs, success={}%",
        harness.min(),
        harness.max(),
        harness.avg(),
        harness.success_rate()
    );
}

/// Test CRSF decode timing
#[test]
fn crsf_decode_timing() {
    let mut harness = TimingTestHarness::new(100); // 100us budget
    let frame = make_crsf_center_frame();

    for _ in 0..1000 {
        let mut decoder = CrsfDecoder::new();

        let start = Instant::now();
        for byte in frame {
            let _ = decoder.decode(byte);
        }
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "CRSF decode max {} μs exceeds 100 μs budget",
        harness.max()
    );
    assert!(
        harness.success_rate() >= TARGET_SUCCESS_RATE,
        "Success rate {} < {}%",
        harness.success_rate(),
        TARGET_SUCCESS_RATE
    );

    println!(
        "CRSF decode: min={} μs, max={} μs, avg={} μs, success={}%",
        harness.min(),
        harness.max(),
        harness.avg(),
        harness.success_rate()
    );
}

/// Test RC server processing timing
#[test]
fn rc_server_process_timing() {
    let mut harness = TimingTestHarness::new(50); // 50us budget
    let mut decoder = SbusDecoder::new();

    // Decode a frame first
    let frame = make_sbus_center_frame();
    let mut channels = None;
    for byte in frame {
        if let Some(ch) = decoder.decode(byte) {
            channels = Some(ch);
        }
    }
    let ch = channels.unwrap();

    for _ in 0..1000 {
        let mut server = RcServer::default();

        let start = Instant::now();
        server.process(&ch);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "RC server process max {} μs exceeds 50 μs budget",
        harness.max()
    );

    println!(
        "RC server process: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test full RC pipeline timing (decode + process)
#[test]
fn full_rc_pipeline_timing() {
    let mut harness = TimingTestHarness::new(200); // 200us budget for full pipeline
    let frame = make_sbus_center_frame();

    for _ in 0..1000 {
        let mut decoder = SbusDecoder::new();
        let mut server = RcServer::default();

        let start = Instant::now();

        // Decode frame
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                // Process channels
                server.process(&ch);
            }
        }

        // Read outputs
        let _throttle = server.throttle();
        let _steering = server.steering();

        let elapsed = start.elapsed().as_micros() as u64;
        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Full RC pipeline max {} μs exceeds 200 μs budget",
        harness.max()
    );
    assert!(
        harness.success_rate() >= TARGET_SUCCESS_RATE,
        "Success rate {} < {}%",
        harness.success_rate(),
        TARGET_SUCCESS_RATE
    );

    println!(
        "Full RC pipeline: min={} μs, max={} μs, avg={} μs, success={}%",
        harness.min(),
        harness.max(),
        harness.avg(),
        harness.success_rate()
    );
}

/// Test comms server update timing
#[test]
fn comms_server_update_timing() {
    let mut harness = TimingTestHarness::new(50); // 50us budget
    let mut config = CommsConfig::default();
    config.heartbeat_interval_ms = 1000;

    let mut server = CommsServer::new(config);
    server.start_connect();
    server.on_connected();

    for i in 0..1000 {
        let start = Instant::now();
        server.update(i);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Comms update max {} μs exceeds 50 μs budget",
        harness.max()
    );

    println!(
        "Comms update: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test telemetry message creation timing
#[test]
fn telemetry_creation_timing() {
    let mut harness = TimingTestHarness::new(20); // 20us budget

    for i in 0..1000 {
        let start = Instant::now();
        let _msg = TelemetryMessage::new(TelemetryType::Motor, i);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Telemetry creation max {} μs exceeds 20 μs budget",
        harness.max()
    );

    println!(
        "Telemetry creation: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test telemetry queue operations timing
#[test]
fn telemetry_queue_timing() {
    let mut harness = TimingTestHarness::new(30); // 30us budget
    let mut server = CommsServer::default();
    server.start_connect();
    server.on_connected();

    for i in 0..1000 {
        // Ensure queue has space
        if server.tx_queue_len() >= 7 {
            let _ = server.pop_telemetry();
        }

        let msg = TelemetryMessage::new(TelemetryType::System, i);

        let start = Instant::now();
        let _ = server.send_telemetry(msg);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Telemetry queue max {} μs exceeds 30 μs budget",
        harness.max()
    );

    println!(
        "Telemetry queue: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test command queue operations timing
#[test]
fn command_queue_timing() {
    let mut harness = TimingTestHarness::new(30); // 30us budget
    let mut server = CommsServer::default();

    for _ in 0..1000 {
        // Ensure queue has space
        if server.rx_queue_len() >= 7 {
            let _ = server.pop_command();
        }

        let start = Instant::now();
        let _ = server.receive_command(ox_services::comms::RemoteCommand::Ping);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Command queue max {} μs exceeds 30 μs budget",
        harness.max()
    );

    println!(
        "Command queue: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test failsafe update timing
#[test]
fn failsafe_update_timing() {
    let mut harness = TimingTestHarness::new(20); // 20us budget
    let mut server = RcServer::default();

    for i in 0..1000 {
        let start = Instant::now();
        server.update(i * 10);
        let elapsed = start.elapsed().as_micros() as u64;

        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Failsafe update max {} μs exceeds 20 μs budget",
        harness.max()
    );

    println!(
        "Failsafe update: min={} μs, max={} μs, avg={} μs",
        harness.min(),
        harness.max(),
        harness.avg()
    );
}

/// Test combined control loop timing (simulated)
#[test]
fn control_loop_combined_timing() {
    let mut harness = TimingTestHarness::new(CONTROL_LOOP_BUDGET_US);

    let frame = make_sbus_center_frame();

    for i in 0..1000 {
        let mut decoder = SbusDecoder::new();
        let mut rc_server = RcServer::default();
        let mut comms = CommsServer::default();
        comms.start_connect();
        comms.on_connected();

        let start = Instant::now();

        // 1. Decode RC frame
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                rc_server.process(&ch);
            }
        }

        // 2. Get control inputs
        let _throttle = rc_server.throttle();
        let _steering = rc_server.steering();
        let _failsafe = rc_server.is_failsafe();

        // 3. Update failsafe timer
        rc_server.update(10);

        // 4. Update comms (heartbeat check)
        comms.update(i);

        // 5. Queue telemetry if needed (every 10 iterations)
        if i % 10 == 0 {
            let msg = TelemetryMessage::new(TelemetryType::Motor, i);
            let _ = comms.send_telemetry(msg);
        }

        let elapsed = start.elapsed().as_micros() as u64;
        harness.record(elapsed);
    }

    assert!(
        harness.meets_target(),
        "Combined loop max {} μs exceeds {} μs budget",
        harness.max(),
        CONTROL_LOOP_BUDGET_US
    );
    assert!(
        harness.success_rate() >= TARGET_SUCCESS_RATE,
        "Success rate {} < {}%",
        harness.success_rate(),
        TARGET_SUCCESS_RATE
    );

    println!(
        "Combined control loop: min={} μs, max={} μs, avg={} μs, success={}%",
        harness.min(),
        harness.max(),
        harness.avg(),
        harness.success_rate()
    );

    // Print timing budget analysis
    println!("\n=== Timing Budget Analysis ===");
    println!("Target: {} μs (1 kHz control loop)", CONTROL_LOOP_BUDGET_US);
    println!("Average used: {} μs ({:.1}% of budget)", harness.avg(), (harness.avg() as f32 / CONTROL_LOOP_BUDGET_US as f32) * 100.0);
    println!("Worst case: {} μs ({:.1}% of budget)", harness.max(), (harness.max() as f32 / CONTROL_LOOP_BUDGET_US as f32) * 100.0);
    println!("Margin remaining: {} μs", CONTROL_LOOP_BUDGET_US.saturating_sub(harness.max()));
}

/// Test harness statistics
#[test]
fn timing_harness_statistics() {
    let mut harness = TimingTestHarness::new(100);

    // Add known values
    harness.record(50);
    harness.record(100);
    harness.record(75);
    harness.record(150); // Over budget

    assert_eq!(harness.count(), 4);
    assert_eq!(harness.min(), 50);
    assert_eq!(harness.max(), 150);
    assert_eq!(harness.avg(), 93); // (50+100+75+150)/4 = 93.75 -> 93
    assert!(!harness.meets_target()); // 150 > 100
    assert!((harness.success_rate() - 75.0).abs() < 1.0); // 3/4 = 75%
}

/// Test empty harness edge cases
#[test]
fn timing_harness_empty() {
    let harness = TimingTestHarness::new(100);

    assert_eq!(harness.count(), 0);
    assert_eq!(harness.min(), 0);
    assert_eq!(harness.max(), 0);
    assert_eq!(harness.avg(), 0);
    assert!(harness.meets_target()); // No failures
    assert_eq!(harness.success_rate(), 0.0);
}
