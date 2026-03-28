//! SPI abstraction

/// SPI device trait for high-speed peripherals (IMUs, displays, etc.)
pub trait SpiDevice {
    type Error;

    /// Transfer data (simultaneous read/write)
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Write bytes only
    fn write(&mut self, data: &[u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Read bytes only (writes zeros)
    fn read(&mut self, buffer: &mut [u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Transfer in place (same buffer for read/write)
    fn transfer_in_place(&mut self, data: &mut [u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}
