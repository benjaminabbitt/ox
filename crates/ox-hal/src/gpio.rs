//! GPIO abstraction

/// Pin mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
    Input,
    Output,
    InputPullUp,
    InputPullDown,
}

/// Output pin trait
pub trait OutputPin {
    type Error;

    fn set_high(&mut self) -> Result<(), Self::Error>;
    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn toggle(&mut self) -> Result<(), Self::Error>;
    fn is_set_high(&self) -> Result<bool, Self::Error>;
}

/// Input pin trait
pub trait InputPin {
    type Error;

    fn is_high(&self) -> Result<bool, Self::Error>;
    fn is_low(&self) -> Result<bool, Self::Error>;
}
