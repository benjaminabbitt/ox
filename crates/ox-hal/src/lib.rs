//! Ox Hardware Abstraction Layer
//!
//! Provides mockable HAL traits for testing without hardware.
//! Real implementations use esp-hal, mocks use in-memory state.

#![no_std]

pub mod gpio;
pub mod pwm;
pub mod i2c;
pub mod encoder;
pub mod motor;

#[cfg(feature = "mock")]
pub mod mock;

// Re-export traits
pub use gpio::{InputPin, OutputPin, PinMode};
pub use pwm::PwmChannel;
pub use i2c::I2cDevice;
pub use encoder::Encoder;
pub use motor::Motor;
