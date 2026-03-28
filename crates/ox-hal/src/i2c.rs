//! I2C abstraction

/// Async I2C device trait
pub trait I2cDevice {
    type Error;

    /// Write bytes to device
    fn write(&mut self, addr: u8, data: &[u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Read bytes from device
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    /// Write then read (common transaction pattern)
    fn write_read(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}
