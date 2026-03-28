//! Mock implementations for testing without hardware
//!
//! Enable with `features = ["mock"]`

use crate::gpio::{InputPin, OutputPin, PinMode};
use crate::pwm::PwmChannel;
use crate::encoder::Encoder;
use crate::motor::{Direction, Motor};

use core::cell::Cell;

/// Mock error type
#[derive(Debug, Clone, Copy)]
pub struct MockError;

/// Mock GPIO pin that tracks state in memory
pub struct MockPin {
    is_output: bool,
    state: Cell<bool>,
}

impl MockPin {
    pub fn new(mode: PinMode) -> Self {
        Self {
            is_output: matches!(mode, PinMode::Output),
            state: Cell::new(false),
        }
    }
}

impl OutputPin for MockPin {
    type Error = MockError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state.set(true);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state.set(false);
        Ok(())
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.state.set(!self.state.get());
        Ok(())
    }

    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state.get())
    }
}

impl InputPin for MockPin {
    type Error = MockError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state.get())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state.get())
    }
}

/// Mock PWM channel
pub struct MockPwm {
    duty: u16,
    enabled: bool,
    frequency: u32,
}

impl MockPwm {
    pub fn new() -> Self {
        Self {
            duty: 0,
            enabled: false,
            frequency: 1000,
        }
    }
}

impl Default for MockPwm {
    fn default() -> Self {
        Self::new()
    }
}

impl PwmChannel for MockPwm {
    type Error = MockError;

    fn set_duty(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.duty = duty;
        Ok(())
    }

    fn get_duty(&self) -> u16 {
        self.duty
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.enabled = false;
        Ok(())
    }

    fn set_frequency(&mut self, freq_hz: u32) -> Result<(), Self::Error> {
        self.frequency = freq_hz;
        Ok(())
    }
}

/// Mock encoder
pub struct MockEncoder {
    position: i32,
    counts_per_rev: u32,
}

impl MockEncoder {
    pub fn new(counts_per_rev: u32) -> Self {
        Self {
            position: 0,
            counts_per_rev,
        }
    }

    /// Simulate encoder movement (for testing)
    pub fn simulate_move(&mut self, counts: i32) {
        self.position = self.position.wrapping_add(counts);
    }
}

impl Encoder for MockEncoder {
    type Error = MockError;

    fn position(&self) -> i32 {
        self.position
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.position = 0;
        Ok(())
    }

    fn set_position(&mut self, pos: i32) -> Result<(), Self::Error> {
        self.position = pos;
        Ok(())
    }

    fn counts_per_rev(&self) -> u32 {
        self.counts_per_rev
    }
}

/// Mock motor
pub struct MockMotor {
    speed: i16,
    direction: Direction,
}

impl MockMotor {
    pub fn new() -> Self {
        Self {
            speed: 0,
            direction: Direction::Coast,
        }
    }
}

impl Default for MockMotor {
    fn default() -> Self {
        Self::new()
    }
}

impl Motor for MockMotor {
    type Error = MockError;

    fn set_speed(&mut self, speed: i16) -> Result<(), Self::Error> {
        self.speed = speed;
        self.direction = if speed > 0 {
            Direction::Forward
        } else if speed < 0 {
            Direction::Reverse
        } else {
            Direction::Brake
        };
        Ok(())
    }

    fn speed(&self) -> i16 {
        self.speed
    }

    fn set_direction(&mut self, dir: Direction) -> Result<(), Self::Error> {
        self.direction = dir;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.speed = 0;
        self.direction = Direction::Brake;
        Ok(())
    }

    fn coast(&mut self) -> Result<(), Self::Error> {
        self.speed = 0;
        self.direction = Direction::Coast;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_pin_toggles() {
        let mut pin = MockPin::new(PinMode::Output);
        assert!(!pin.state.get());
        pin.set_high().unwrap();
        assert!(pin.state.get());
        pin.toggle().unwrap();
        assert!(!pin.state.get());
    }

    #[test]
    fn mock_pwm_sets_duty() {
        let mut pwm = MockPwm::new();
        pwm.set_duty(32768).unwrap();
        assert_eq!(pwm.get_duty(), 32768);
    }

    #[test]
    fn mock_encoder_tracks_position() {
        let mut enc = MockEncoder::new(1000);
        enc.simulate_move(100);
        assert_eq!(enc.position(), 100);
        enc.simulate_move(-50);
        assert_eq!(enc.position(), 50);
    }

    #[test]
    fn mock_motor_sets_speed() {
        let mut motor = MockMotor::new();
        motor.set_speed(1000).unwrap();
        assert_eq!(motor.speed(), 1000);
        motor.stop().unwrap();
        assert_eq!(motor.speed(), 0);
    }
}
