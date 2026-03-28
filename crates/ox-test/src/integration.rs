//! Integration test utilities
//!
//! Helpers for testing component interactions.

/// RC system integration test helper
///
/// Tests the full RC pipeline: decoder → RcServer → motor commands
pub struct RcTestHarness {
    /// Accumulated motor commands from RC
    pub motor_commands: Vec<(i16, i16)>,
    /// Number of failsafe events
    pub failsafe_count: u32,
    /// Number of valid frames processed
    pub valid_frame_count: u32,
}

impl RcTestHarness {
    pub fn new() -> Self {
        Self {
            motor_commands: Vec::new(),
            failsafe_count: 0,
            valid_frame_count: 0,
        }
    }

    /// Record a motor command
    pub fn record_command(&mut self, left: i16, right: i16) {
        self.motor_commands.push((left, right));
    }

    /// Record a failsafe event
    pub fn record_failsafe(&mut self) {
        self.failsafe_count += 1;
    }

    /// Record a valid frame
    pub fn record_valid_frame(&mut self) {
        self.valid_frame_count += 1;
    }

    /// Get the last motor command
    pub fn last_command(&self) -> Option<(i16, i16)> {
        self.motor_commands.last().copied()
    }

    /// Check if commands are within expected differential drive behavior
    pub fn verify_differential_drive(&self, throttle: f32, steering: f32) -> bool {
        if let Some((left, right)) = self.last_command() {
            // Expected: left = (throttle + steering) * 1000
            //           right = (throttle - steering) * 1000
            let expected_left = ((throttle + steering) * 1000.0) as i16;
            let expected_right = ((throttle - steering) * 1000.0) as i16;

            // Allow some tolerance for floating point
            let tolerance = 50;
            (left - expected_left).abs() < tolerance && (right - expected_right).abs() < tolerance
        } else {
            false
        }
    }
}

impl Default for RcTestHarness {
    fn default() -> Self {
        Self::new()
    }
}

/// Comms system integration test helper
pub struct CommsTestHarness {
    /// Connection state transitions
    pub state_transitions: Vec<&'static str>,
    /// Telemetry messages sent
    pub telemetry_sent: u32,
    /// Commands received
    pub commands_received: u32,
}

impl CommsTestHarness {
    pub fn new() -> Self {
        Self {
            state_transitions: Vec::new(),
            telemetry_sent: 0,
            commands_received: 0,
        }
    }

    pub fn record_state(&mut self, state: &'static str) {
        self.state_transitions.push(state);
    }

    pub fn record_telemetry(&mut self) {
        self.telemetry_sent += 1;
    }

    pub fn record_command(&mut self) {
        self.commands_received += 1;
    }

    /// Verify expected state transition sequence
    pub fn verify_transitions(&self, expected: &[&str]) -> bool {
        self.state_transitions == expected
    }
}

impl Default for CommsTestHarness {
    fn default() -> Self {
        Self::new()
    }
}

/// Timing test helper
pub struct TimingTestHarness {
    samples: Vec<u64>,
    target_us: u64,
}

impl TimingTestHarness {
    pub fn new(target_us: u64) -> Self {
        Self {
            samples: Vec::new(),
            target_us,
        }
    }

    pub fn record(&mut self, duration_us: u64) {
        self.samples.push(duration_us);
    }

    pub fn min(&self) -> u64 {
        self.samples.iter().copied().min().unwrap_or(0)
    }

    pub fn max(&self) -> u64 {
        self.samples.iter().copied().max().unwrap_or(0)
    }

    pub fn avg(&self) -> u64 {
        if self.samples.is_empty() {
            0
        } else {
            self.samples.iter().sum::<u64>() / self.samples.len() as u64
        }
    }

    pub fn meets_target(&self) -> bool {
        self.max() <= self.target_us
    }

    pub fn count(&self) -> usize {
        self.samples.len()
    }

    /// Get percentage of samples meeting target
    pub fn success_rate(&self) -> f32 {
        if self.samples.is_empty() {
            return 0.0;
        }
        let passing = self.samples.iter().filter(|&&s| s <= self.target_us).count();
        passing as f32 / self.samples.len() as f32 * 100.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rc_harness_records_commands() {
        let mut harness = RcTestHarness::new();
        harness.record_command(500, 500);
        harness.record_command(750, 250);

        assert_eq!(harness.motor_commands.len(), 2);
        assert_eq!(harness.last_command(), Some((750, 250)));
    }

    #[test]
    fn rc_harness_verifies_differential_drive() {
        let mut harness = RcTestHarness::new();

        // throttle=0.5, steering=0.0 → left=500, right=500
        harness.record_command(500, 500);
        assert!(harness.verify_differential_drive(0.5, 0.0));

        // throttle=0.5, steering=0.25 → left=750, right=250
        harness.record_command(750, 250);
        assert!(harness.verify_differential_drive(0.5, 0.25));
    }

    #[test]
    fn comms_harness_tracks_transitions() {
        let mut harness = CommsTestHarness::new();
        harness.record_state("Disconnected");
        harness.record_state("Connecting");
        harness.record_state("Connected");

        assert!(harness.verify_transitions(&["Disconnected", "Connecting", "Connected"]));
    }

    #[test]
    fn timing_harness_calculates_stats() {
        let mut harness = TimingTestHarness::new(1000);
        harness.record(500);
        harness.record(800);
        harness.record(600);

        assert_eq!(harness.min(), 500);
        assert_eq!(harness.max(), 800);
        assert_eq!(harness.avg(), 633);
        assert!(harness.meets_target());
        assert_eq!(harness.success_rate(), 100.0);
    }

    #[test]
    fn timing_harness_detects_failures() {
        let mut harness = TimingTestHarness::new(1000);
        harness.record(500);
        harness.record(1500); // Exceeds target
        harness.record(800);

        assert!(!harness.meets_target());
        assert!((harness.success_rate() - 66.66).abs() < 1.0);
    }
}
