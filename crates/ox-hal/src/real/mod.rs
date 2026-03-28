//! Real hardware implementations for ESP32
//!
//! These wrap esp-hal peripherals and implement ox-hal traits.
//!
//! # Usage Pattern
//!
//! The real HAL wrappers accept already-configured esp-hal peripherals.
//! This keeps configuration flexible while providing trait implementations.
//!
//! ```ignore
//! // Configure esp-hal peripheral
//! let output = Output::new(peripherals.GPIO2, Level::Low);
//! // Wrap in ox-hal type
//! let pin = EspOutputPin::new(output);
//! // Use via trait
//! pin.set_high()?;
//! ```

pub mod gpio;
pub mod pwm;
pub mod i2c;
pub mod spi;

pub use gpio::{EspOutputPin, EspInputPin};
pub use pwm::EspPwm;
pub use i2c::EspI2c;
pub use spi::EspSpi;

/// Error type for real HAL operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HalError {
    /// Operation not supported
    NotSupported,
    /// Bus error (I2C, SPI)
    BusError,
    /// Invalid configuration
    InvalidConfig,
    /// Timeout
    Timeout,
}
