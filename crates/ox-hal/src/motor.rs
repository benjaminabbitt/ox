//! Motor driver abstraction

/// Motor direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Brake,
    Coast,
}

/// DC Motor driver trait
pub trait Motor {
    type Error;

    /// Set motor speed (-32768 to 32767, negative = reverse)
    fn set_speed(&mut self, speed: i16) -> Result<(), Self::Error>;

    /// Get current speed setting
    fn speed(&self) -> i16;

    /// Set direction explicitly
    fn set_direction(&mut self, dir: Direction) -> Result<(), Self::Error>;

    /// Emergency stop (brake)
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Coast (disable driver)
    fn coast(&mut self) -> Result<(), Self::Error>;
}
