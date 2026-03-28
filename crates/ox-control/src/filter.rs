//! Signal filters
//!
//! - Low-pass filter
//! - Complementary filter (for IMU fusion)

/// First-order low-pass filter
pub struct LowPassFilter {
    alpha: f32,
    value: f32,
    initialized: bool,
}

impl LowPassFilter {
    /// Create a new low-pass filter
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate_hz` - Sample rate in Hz
    pub fn new(cutoff_hz: f32, sample_rate_hz: f32) -> Self {
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        let dt = 1.0 / sample_rate_hz;
        let alpha = dt / (rc + dt);

        Self {
            alpha,
            value: 0.0,
            initialized: false,
        }
    }

    /// Create with explicit alpha value
    pub fn with_alpha(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            value: 0.0,
            initialized: false,
        }
    }

    /// Update the filter with a new sample
    pub fn update(&mut self, input: f32) -> f32 {
        if !self.initialized {
            self.value = input;
            self.initialized = true;
        } else {
            self.value = self.alpha * input + (1.0 - self.alpha) * self.value;
        }
        self.value
    }

    /// Get current filtered value
    pub fn value(&self) -> f32 {
        self.value
    }

    /// Reset the filter
    pub fn reset(&mut self) {
        self.value = 0.0;
        self.initialized = false;
    }
}

/// Complementary filter for IMU sensor fusion
///
/// Combines accelerometer (low frequency) and gyroscope (high frequency)
/// readings to get stable angle estimation.
pub struct ComplementaryFilter {
    alpha: f32, // Weight for gyro (typically 0.98)
    angle: f32,
}

impl ComplementaryFilter {
    /// Create a new complementary filter
    ///
    /// # Arguments
    /// * `alpha` - Weight for gyroscope (0.0-1.0, typically 0.98)
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            angle: 0.0,
        }
    }

    /// Update the filter
    ///
    /// # Arguments
    /// * `accel_angle` - Angle from accelerometer (degrees)
    /// * `gyro_rate` - Angular rate from gyroscope (degrees/sec)
    /// * `dt` - Time step in seconds
    pub fn update(&mut self, accel_angle: f32, gyro_rate: f32, dt: f32) -> f32 {
        // Gyro integration (high frequency)
        let gyro_angle = self.angle + gyro_rate * dt;

        // Complementary fusion
        self.angle = self.alpha * gyro_angle + (1.0 - self.alpha) * accel_angle;
        self.angle
    }

    /// Get current angle estimate
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Set angle directly (for initialization)
    pub fn set_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    /// Reset to zero
    pub fn reset(&mut self) {
        self.angle = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // === LowPassFilter Tests ===

    #[test]
    fn lowpass_smooths_signal() {
        let mut filter = LowPassFilter::with_alpha(0.1);
        filter.update(0.0);
        filter.update(100.0);
        // Should be smoothed, not jump to 100
        assert!(filter.value() < 50.0);
    }

    #[test]
    fn lowpass_first_sample_initializes() {
        let mut filter = LowPassFilter::with_alpha(0.5);
        let output = filter.update(42.0);
        assert_eq!(output, 42.0, "First sample should initialize to input");
    }

    #[test]
    fn lowpass_converges_to_constant_input() {
        let mut filter = LowPassFilter::with_alpha(0.3);
        let target = 100.0;

        for _ in 0..50 {
            filter.update(target);
        }

        let error = (filter.value() - target).abs();
        assert!(error < 1.0, "Should converge to constant input, error={}", error);
    }

    #[test]
    fn lowpass_alpha_zero_ignores_new_samples() {
        let mut filter = LowPassFilter::with_alpha(0.0);
        filter.update(100.0); // Initialize
        filter.update(0.0);
        filter.update(0.0);

        assert_eq!(filter.value(), 100.0, "Alpha=0 should ignore new samples");
    }

    #[test]
    fn lowpass_alpha_one_tracks_immediately() {
        let mut filter = LowPassFilter::with_alpha(1.0);
        filter.update(100.0);
        filter.update(50.0);

        assert_eq!(filter.value(), 50.0, "Alpha=1 should track immediately");
    }

    #[test]
    fn lowpass_reset_clears_state() {
        let mut filter = LowPassFilter::with_alpha(0.5);
        filter.update(100.0);
        filter.reset();

        assert_eq!(filter.value(), 0.0);
        // Next update should reinitialize
        let output = filter.update(42.0);
        assert_eq!(output, 42.0);
    }

    #[test]
    fn lowpass_rejects_high_frequency_noise() {
        let mut filter = LowPassFilter::with_alpha(0.1);

        // Alternating signal (high frequency noise)
        let mut sum = 0.0;
        for i in 0..100 {
            let input = if i % 2 == 0 { 100.0 } else { -100.0 };
            sum += filter.update(input);
        }

        // Average should be close to 0 (noise canceled)
        let avg = sum / 100.0;
        assert!(avg.abs() < 20.0, "Should filter high-frequency noise, avg={}", avg);
    }

    #[test]
    fn lowpass_from_cutoff_frequency() {
        // 10 Hz cutoff at 100 Hz sample rate
        let filter = LowPassFilter::new(10.0, 100.0);
        // Alpha should be reasonable (not 0 or 1)
        // For this config, alpha ≈ 0.39
        assert!(filter.alpha > 0.1 && filter.alpha < 0.9);
    }

    #[test]
    fn lowpass_alpha_clamped() {
        let filter1 = LowPassFilter::with_alpha(-1.0);
        assert_eq!(filter1.alpha, 0.0);

        let filter2 = LowPassFilter::with_alpha(2.0);
        assert_eq!(filter2.alpha, 1.0);
    }

    // === ComplementaryFilter Tests ===

    #[test]
    fn complementary_tracks_gyro_short_term() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(0.0);

        // Simulate rotation
        for _ in 0..10 {
            filter.update(0.0, 10.0, 0.01); // Gyro says rotating, accel says level
        }

        // Should track gyro more than accel
        assert!(filter.angle() > 0.5);
    }

    #[test]
    fn complementary_tracks_accel_long_term() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(0.0);

        // Many updates with accel saying 45°, gyro saying no rotation
        for _ in 0..500 {
            filter.update(45.0, 0.0, 0.01);
        }

        // Should converge toward accel reading
        let error = (filter.angle() - 45.0).abs();
        assert!(error < 5.0, "Should converge to accel, error={}", error);
    }

    #[test]
    fn complementary_zero_alpha_tracks_accel_only() {
        let mut filter = ComplementaryFilter::new(0.0);
        filter.set_angle(0.0);

        filter.update(45.0, 100.0, 0.01); // Accel says 45, gyro says fast rotation

        // Should immediately jump to accel
        assert!((filter.angle() - 45.0).abs() < 0.1);
    }

    #[test]
    fn complementary_one_alpha_tracks_gyro_only() {
        let mut filter = ComplementaryFilter::new(1.0);
        filter.set_angle(0.0);

        filter.update(45.0, 10.0, 0.1); // dt=0.1s, rate=10°/s -> +1°

        // Should only integrate gyro
        assert!((filter.angle() - 1.0).abs() < 0.1);
    }

    #[test]
    fn complementary_reset_clears_angle() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(90.0);
        filter.reset();

        assert_eq!(filter.angle(), 0.0);
    }

    #[test]
    fn complementary_set_angle_works() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(123.0);

        assert_eq!(filter.angle(), 123.0);
    }

    #[test]
    fn complementary_handles_negative_angles() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(-45.0);

        filter.update(-50.0, -5.0, 0.1);

        assert!(filter.angle() < -45.0, "Should handle negative angles");
    }

    #[test]
    fn complementary_alpha_clamped() {
        let filter1 = ComplementaryFilter::new(-0.5);
        assert_eq!(filter1.alpha, 0.0);

        let filter2 = ComplementaryFilter::new(1.5);
        assert_eq!(filter2.alpha, 1.0);
    }

    #[test]
    fn complementary_gyro_drift_corrected_by_accel() {
        let mut filter = ComplementaryFilter::new(0.98);
        filter.set_angle(0.0);

        // Gyro has drift of 0.1°/s, accel is stable at 0
        for _ in 0..1000 {
            filter.update(0.0, 0.1, 0.01);
        }

        // Drift should be bounded by accel correction
        assert!(filter.angle().abs() < 5.0, "Gyro drift should be corrected, angle={}", filter.angle());
    }
}
