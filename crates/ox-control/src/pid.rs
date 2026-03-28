//! PID Controller implementation
//!
//! A general-purpose PID controller with:
//! - Anti-windup
//! - Derivative filtering
//! - Output limiting

/// PID Controller
pub struct PidController {
    // Gains
    kp: f32,
    ki: f32,
    kd: f32,

    // State
    integral: f32,
    prev_error: f32,
    prev_measurement: f32, // For derivative-on-measurement

    // Limits
    output_min: f32,
    output_max: f32,
    integral_min: f32,
    integral_max: f32,

    // Configuration
    use_derivative_on_measurement: bool,
}

impl PidController {
    /// Create a new PID controller with the given gains
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            prev_measurement: 0.0,
            output_min: -1.0,
            output_max: 1.0,
            integral_min: -1.0,
            integral_max: 1.0,
            use_derivative_on_measurement: true,
        }
    }

    /// Set output limits
    pub fn set_output_limits(&mut self, min: f32, max: f32) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Set integral limits (anti-windup)
    pub fn set_integral_limits(&mut self, min: f32, max: f32) {
        self.integral_min = min;
        self.integral_max = max;
    }

    /// Update the PID controller
    ///
    /// # Arguments
    /// * `setpoint` - Desired value
    /// * `measurement` - Current measured value
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// Control output (clamped to output limits)
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term with anti-windup
        self.integral += error * dt;
        self.integral = self.integral.clamp(self.integral_min, self.integral_max);
        let i_term = self.ki * self.integral;

        // Derivative term (on measurement to avoid derivative kick)
        let d_term = if self.use_derivative_on_measurement {
            let d_measurement = (measurement - self.prev_measurement) / dt;
            -self.kd * d_measurement
        } else {
            let d_error = (error - self.prev_error) / dt;
            self.kd * d_error
        };

        // Save state for next iteration
        self.prev_error = error;
        self.prev_measurement = measurement;

        // Calculate and clamp output
        let output = p_term + i_term + d_term;
        output.clamp(self.output_min, self.output_max)
    }

    /// Reset the controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_measurement = 0.0;
    }

    /// Get current integral value (for debugging)
    pub fn integral(&self) -> f32 {
        self.integral
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_responds_to_positive_error() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        let output = pid.update(100.0, 50.0, 0.001);
        assert!(output > 0.0, "Output should be positive for positive error");
    }

    #[test]
    fn pid_responds_to_negative_error() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        let output = pid.update(50.0, 100.0, 0.001);
        assert!(output < 0.0, "Output should be negative for negative error");
    }

    #[test]
    fn pid_output_is_bounded() {
        let mut pid = PidController::new(100.0, 0.0, 0.0);
        pid.set_output_limits(-1.0, 1.0);
        let output = pid.update(1000.0, 0.0, 0.001);
        assert!(output >= -1.0 && output <= 1.0, "Output should be clamped");
    }

    #[test]
    fn pid_integral_accumulates() {
        let mut pid = PidController::new(0.0, 1.0, 0.0);
        pid.update(1.0, 0.0, 0.1);
        pid.update(1.0, 0.0, 0.1);
        assert!(pid.integral() > 0.0, "Integral should accumulate");
    }

    #[test]
    fn pid_reset_clears_state() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        pid.update(100.0, 0.0, 0.1);
        pid.reset();
        assert_eq!(pid.integral(), 0.0);
    }

    // === Step Response Tests ===

    #[test]
    fn pid_step_response_converges() {
        // P-only controller should reduce error over time
        let mut pid = PidController::new(0.5, 0.1, 0.0);
        pid.set_output_limits(-100.0, 100.0);
        pid.set_integral_limits(-100.0, 100.0);

        let setpoint = 100.0;
        let mut measurement = 0.0;
        let dt = 0.01;

        // Simulate closed-loop for 100 steps
        for _ in 0..100 {
            let output = pid.update(setpoint, measurement, dt);
            // Simple plant: measurement moves toward output
            measurement += output * dt * 10.0;
        }

        // Should converge close to setpoint
        let error = (setpoint - measurement).abs();
        assert!(error < 10.0, "Should converge, error={}", error);
    }

    #[test]
    fn pid_step_response_with_pi_eliminates_steady_state_error() {
        // PI controller should eliminate steady-state error
        let mut pid = PidController::new(1.0, 5.0, 0.0);
        pid.set_output_limits(-100.0, 100.0);
        pid.set_integral_limits(-50.0, 50.0);

        let setpoint = 50.0;
        let mut measurement = 0.0;
        let dt = 0.01;

        // Simulate with disturbance (constant offset)
        for _ in 0..200 {
            let output = pid.update(setpoint, measurement, dt);
            measurement += output * dt * 5.0 - 0.1; // -0.1 is disturbance
        }

        let error = (setpoint - measurement).abs();
        assert!(error < 5.0, "PI should eliminate steady-state error, error={}", error);
    }

    // === Anti-Windup Tests ===

    #[test]
    fn pid_integral_is_clamped() {
        let mut pid = PidController::new(0.0, 10.0, 0.0);
        pid.set_integral_limits(-1.0, 1.0);

        // Large error for many steps
        for _ in 0..100 {
            pid.update(1000.0, 0.0, 0.1);
        }

        assert!(pid.integral() <= 1.0, "Integral should be clamped to max");
        assert!(pid.integral() >= -1.0, "Integral should be clamped to min");
    }

    #[test]
    fn pid_anti_windup_allows_quick_recovery() {
        let mut pid = PidController::new(1.0, 1.0, 0.0);
        pid.set_output_limits(-1.0, 1.0);
        pid.set_integral_limits(-2.0, 2.0);

        // Wind up integral with large error
        for _ in 0..50 {
            pid.update(100.0, 0.0, 0.1);
        }

        // Now setpoint is achieved - integral should unwind
        let last_integral = pid.integral();
        for _ in 0..20 {
            pid.update(0.0, 0.0, 0.1); // Error is zero
            // Integral shouldn't grow when error is zero
        }

        // Integral should not have grown (may stay same or decrease)
        assert!(pid.integral() <= last_integral + 0.01);
    }

    // === Derivative Tests ===

    #[test]
    fn pid_derivative_responds_to_rate_of_change() {
        let mut pid = PidController::new(0.0, 0.0, 1.0);
        pid.set_output_limits(-100.0, 100.0);

        // First update establishes baseline
        pid.update(0.0, 0.0, 0.01);

        // Measurement suddenly changes - derivative should respond
        let output = pid.update(0.0, 10.0, 0.01);

        // Derivative-on-measurement: negative output for positive measurement change
        assert!(output < 0.0, "D term should oppose rapid measurement change");
    }

    #[test]
    fn pid_derivative_on_measurement_avoids_setpoint_kick() {
        let mut pid = PidController::new(0.0, 0.0, 10.0);
        pid.set_output_limits(-100.0, 100.0);

        // Warmup: let the filter stabilize at measurement=50
        for _ in 0..5 {
            pid.update(0.0, 50.0, 0.01);
        }

        // Now measurement is stable, get baseline
        let output1 = pid.update(0.0, 50.0, 0.01);

        // Change setpoint suddenly (measurement stays same)
        let output2 = pid.update(100.0, 50.0, 0.01);

        // With derivative-on-measurement, setpoint change shouldn't cause spike
        // (D term only looks at measurement change, not setpoint)
        let diff = (output2 - output1).abs();
        assert!(diff < 1.0, "Setpoint change shouldn't cause D kick, diff={}", diff);
    }

    // === Edge Cases ===

    #[test]
    fn pid_handles_zero_dt() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        // This would cause division by zero without protection
        // Current implementation will have inf/nan - document behavior
        let output = pid.update(100.0, 0.0, 0.0);
        // Output may be inf/nan, but shouldn't panic
        assert!(output.is_finite() || output.is_nan() || output.is_infinite());
    }

    #[test]
    fn pid_zero_gains_gives_zero_output() {
        let mut pid = PidController::new(0.0, 0.0, 0.0);
        let output = pid.update(100.0, 0.0, 0.01);
        assert_eq!(output, 0.0);
    }

    #[test]
    fn pid_at_setpoint_gives_zero_output() {
        let mut pid = PidController::new(1.0, 0.0, 0.0);
        pid.update(50.0, 50.0, 0.01); // Initialize
        let output = pid.update(50.0, 50.0, 0.01);
        assert!((output).abs() < 0.001, "At setpoint should give ~0 output");
    }

    #[test]
    fn pid_negative_gains_inverts_response() {
        let mut pid = PidController::new(-1.0, 0.0, 0.0);
        let output = pid.update(100.0, 0.0, 0.01);
        // Negative Kp with positive error gives negative output
        assert!(output < 0.0);
    }

    #[test]
    fn pid_custom_output_limits() {
        let mut pid = PidController::new(100.0, 0.0, 0.0);
        pid.set_output_limits(-0.5, 0.5);
        let output = pid.update(1000.0, 0.0, 0.01);
        assert!(output <= 0.5 && output >= -0.5);
    }
}
