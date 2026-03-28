//! Quadrature encoder abstraction

/// Quadrature encoder trait
pub trait Encoder {
    type Error;

    /// Get current position count
    fn position(&self) -> i32;

    /// Reset position to zero
    fn reset(&mut self) -> Result<(), Self::Error>;

    /// Set position to specific value
    fn set_position(&mut self, pos: i32) -> Result<(), Self::Error>;

    /// Get counts per revolution (for velocity calculation)
    fn counts_per_rev(&self) -> u32;
}
