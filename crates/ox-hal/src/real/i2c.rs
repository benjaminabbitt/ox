//! Real I2C implementation for ESP32
//!
//! Wraps esp-hal I2C peripheral with async support.

use crate::i2c::I2cDevice;
use super::HalError;

use esp_hal::i2c::master::I2c;
use esp_hal::Async;

/// I2C wrapper for ESP32
///
/// Wraps an async I2C instance from esp-hal.
pub struct EspI2c<'d> {
    i2c: I2c<'d, Async>,
}

impl<'d> EspI2c<'d> {
    /// Create from an already-configured esp-hal I2C instance
    ///
    /// # Example
    /// ```ignore
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())
    ///     .unwrap()
    ///     .with_sda(peripherals.GPIO4)
    ///     .with_scl(peripherals.GPIO5)
    ///     .into_async();
    /// let esp_i2c = EspI2c::new(i2c);
    /// ```
    pub fn new(i2c: I2c<'d, Async>) -> Self {
        Self { i2c }
    }
}

impl I2cDevice for EspI2c<'_> {
    type Error = HalError;

    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.i2c
            .write(addr, data)
            .await
            .map_err(|_| HalError::BusError)
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c
            .read(addr, buffer)
            .await
            .map_err(|_| HalError::BusError)
    }

    async fn write_read(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(addr, write, read)
            .await
            .map_err(|_| HalError::BusError)
    }
}
