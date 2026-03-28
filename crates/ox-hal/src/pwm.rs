//! PWM abstraction

/// PWM channel trait for motor control
pub trait PwmChannel {
    type Error;

    /// Set duty cycle (0-65535 = 0-100%)
    fn set_duty(&mut self, duty: u16) -> Result<(), Self::Error>;

    /// Get current duty cycle
    fn get_duty(&self) -> u16;

    /// Enable PWM output
    fn enable(&mut self) -> Result<(), Self::Error>;

    /// Disable PWM output
    fn disable(&mut self) -> Result<(), Self::Error>;

    /// Set PWM frequency in Hz
    fn set_frequency(&mut self, freq_hz: u32) -> Result<(), Self::Error>;
}
