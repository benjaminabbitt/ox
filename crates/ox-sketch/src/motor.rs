//! Motor control for driving DC motors
//!
//! Provides simple interfaces for controlling motors.
//! For Phase 0.5, this offers basic digital on/off control.
//! PWM control can be added in a future phase.

use crate::gpio::OutputHandle;

/// Differential drive controller for tank-style or car-style robots
///
/// Controls two motors (left and right) for differential steering.
/// Uses simple on/off control (no PWM speed control in Phase 0.5).
///
/// # Example
/// ```ignore
/// let left_fwd = bot.output(bot.gpio4());
/// let left_rev = bot.output(bot.gpio5());
/// let right_fwd = bot.output(bot.gpio6());
/// let right_rev = bot.output(bot.gpio7());
///
/// let mut drive = DifferentialDrive::new(left_fwd, left_rev, right_fwd, right_rev);
///
/// // Arcade drive: throttle + steering
/// drive.arcade(0.5, 0.2); // forward, slight right turn
/// ```
pub struct DifferentialDrive {
    left_fwd: OutputHandle,
    left_rev: OutputHandle,
    right_fwd: OutputHandle,
    right_rev: OutputHandle,
    deadzone: f32,
}

impl DifferentialDrive {
    /// Create a new differential drive controller
    ///
    /// Requires 4 GPIO pins for H-bridge control:
    /// - left_fwd: Left motor forward
    /// - left_rev: Left motor reverse
    /// - right_fwd: Right motor forward
    /// - right_rev: Right motor reverse
    pub fn new(
        left_fwd: OutputHandle,
        left_rev: OutputHandle,
        right_fwd: OutputHandle,
        right_rev: OutputHandle,
    ) -> Self {
        Self {
            left_fwd,
            left_rev,
            right_fwd,
            right_rev,
            deadzone: 0.1,
        }
    }

    /// Set the deadzone threshold (default 0.1)
    ///
    /// Values below this magnitude are treated as zero.
    pub fn set_deadzone(&mut self, deadzone: f32) {
        self.deadzone = deadzone.abs();
    }

    /// Arcade drive - throttle and steering
    ///
    /// - throttle: forward/backward [-1.0, 1.0]
    /// - steering: left/right [-1.0, 1.0] (positive = turn right)
    pub fn arcade(&mut self, throttle: f32, steering: f32) {
        let throttle = throttle.clamp(-1.0, 1.0);
        let steering = steering.clamp(-1.0, 1.0);

        // Mix throttle and steering
        let left = (throttle + steering).clamp(-1.0, 1.0);
        let right = (throttle - steering).clamp(-1.0, 1.0);

        self.tank(left, right);
    }

    /// Tank drive - control left and right independently
    ///
    /// Both values in range [-1.0, 1.0]
    pub fn tank(&mut self, left: f32, right: f32) {
        // Apply deadzone and drive left motor
        if left > self.deadzone {
            self.left_fwd.high();
            self.left_rev.low();
        } else if left < -self.deadzone {
            self.left_fwd.low();
            self.left_rev.high();
        } else {
            self.left_fwd.low();
            self.left_rev.low();
        }

        // Apply deadzone and drive right motor
        if right > self.deadzone {
            self.right_fwd.high();
            self.right_rev.low();
        } else if right < -self.deadzone {
            self.right_fwd.low();
            self.right_rev.high();
        } else {
            self.right_fwd.low();
            self.right_rev.low();
        }
    }

    /// Drive with RC input
    ///
    /// Convenience method for using with RcInput
    pub fn drive(&mut self, throttle: f32, steering: f32) {
        self.arcade(throttle, steering);
    }

    /// Stop both motors (coast)
    pub fn stop(&mut self) {
        self.left_fwd.low();
        self.left_rev.low();
        self.right_fwd.low();
        self.right_rev.low();
    }

    /// Brake both motors (both pins high)
    pub fn brake(&mut self) {
        self.left_fwd.high();
        self.left_rev.high();
        self.right_fwd.high();
        self.right_rev.high();
    }
}

/// Single motor with direction control
///
/// Uses two GPIO pins for H-bridge control.
pub struct Motor {
    fwd: OutputHandle,
    rev: OutputHandle,
    deadzone: f32,
}

impl Motor {
    /// Create a new motor controller
    pub fn new(fwd: OutputHandle, rev: OutputHandle) -> Self {
        Self {
            fwd,
            rev,
            deadzone: 0.1,
        }
    }

    /// Set motor speed [-1.0, 1.0]
    ///
    /// Positive = forward, negative = reverse.
    /// Values within deadzone are treated as stop.
    pub fn set(&mut self, speed: f32) {
        if speed > self.deadzone {
            self.fwd.high();
            self.rev.low();
        } else if speed < -self.deadzone {
            self.fwd.low();
            self.rev.high();
        } else {
            self.fwd.low();
            self.rev.low();
        }
    }

    /// Stop the motor
    pub fn stop(&mut self) {
        self.fwd.low();
        self.rev.low();
    }

    /// Brake the motor
    pub fn brake(&mut self) {
        self.fwd.high();
        self.rev.high();
    }
}
