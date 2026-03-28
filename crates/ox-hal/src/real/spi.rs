//! Real SPI implementation for ESP32
//!
//! Wraps esp-hal SPI peripheral with async support.

use crate::spi::SpiDevice;
use super::HalError;

use esp_hal::spi::master::Spi;
use esp_hal::Async;
use embedded_hal_async::spi::SpiBus;

/// SPI wrapper for ESP32
///
/// Wraps an async SPI instance from esp-hal.
pub struct EspSpi<'d> {
    spi: Spi<'d, Async>,
}

impl<'d> EspSpi<'d> {
    /// Create from an already-configured esp-hal SPI instance
    ///
    /// # Example
    /// ```ignore
    /// let spi = Spi::new(peripherals.SPI2, Config::default())
    ///     .unwrap()
    ///     .with_sck(peripherals.GPIO6)
    ///     .with_mosi(peripherals.GPIO7)
    ///     .with_miso(peripherals.GPIO2)
    ///     .into_async();
    /// let esp_spi = EspSpi::new(spi);
    /// ```
    pub fn new(spi: Spi<'d, Async>) -> Self {
        Self { spi }
    }
}

impl SpiDevice for EspSpi<'_> {
    type Error = HalError;

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        SpiBus::transfer(&mut self.spi, read, write)
            .await
            .map_err(|_| HalError::BusError)
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        SpiBus::write(&mut self.spi, data)
            .await
            .map_err(|_| HalError::BusError)
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        SpiBus::read(&mut self.spi, buffer)
            .await
            .map_err(|_| HalError::BusError)
    }

    async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        SpiBus::transfer_in_place(&mut self.spi, data)
            .await
            .map_err(|_| HalError::BusError)
    }
}
