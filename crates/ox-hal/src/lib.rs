//! Ox Hardware Abstraction Layer
//!
//! Provides mockable HAL traits for testing without hardware.
//! Real implementations use esp-hal, mocks use in-memory state.
//!
//! # Features
//!
//! - `mock` - Enable mock implementations for host testing
//! - `esp32c3` - Enable real HAL for ESP32-C3
//! - `esp32c6` - Enable real HAL for ESP32-C6
//! - `esp32h2` - Enable real HAL for ESP32-H2
//! - `esp32s3` - Enable real HAL for ESP32-S3

#![no_std]

pub mod gpio;
pub mod pwm;
pub mod i2c;
pub mod spi;
pub mod encoder;
pub mod motor;

#[cfg(any(feature = "mock", test))]
pub mod mock;

#[cfg(any(feature = "esp32c3", feature = "esp32c6", feature = "esp32h2", feature = "esp32s3"))]
pub mod real;

// Re-export traits
pub use gpio::{InputPin, OutputPin, PinMode};
pub use pwm::PwmChannel;
pub use i2c::I2cDevice;
pub use spi::SpiDevice;
pub use encoder::Encoder;
pub use motor::{Direction, Motor};

// Re-export real implementations when available
#[cfg(any(feature = "esp32c3", feature = "esp32c6", feature = "esp32h2", feature = "esp32s3"))]
pub use real::{EspOutputPin, EspInputPin, EspPwm, EspI2c, EspSpi, HalError};
