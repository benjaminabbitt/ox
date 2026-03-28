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

    #[test]
    fn lowpass_smooths_signal() {
        let mut filter = LowPassFilter::with_alpha(0.1);
        filter.update(0.0);
        filter.update(100.0);
        // Should be smoothed, not jump to 100
        assert!(filter.value() < 50.0);
    }

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
}
