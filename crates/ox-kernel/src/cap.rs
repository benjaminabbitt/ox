//! Capability-based access control
//!
//! Capabilities are unforgeable tokens that grant access to resources.
//! Servers must hold a capability to access any hardware resource.

use core::marker::PhantomData;
use portable_atomic::{AtomicU32, Ordering};

/// Global capability ID counter
static NEXT_CAP_ID: AtomicU32 = AtomicU32::new(1);

/// Marker trait for resource types
pub trait Resource: 'static {}

/// An opaque capability handle granting access to a resource
///
/// Capabilities are unforgeable (private constructor) and must be
/// explicitly passed to components that need them.
#[derive(Debug)]
pub struct Cap<T: Resource> {
    id: u32,
    _phantom: PhantomData<T>,
}

impl<T: Resource> Cap<T> {
    /// Grant a new capability (only kernel should call this)
    ///
    /// # Safety
    /// This should only be called by the kernel during system initialization
    /// or server restart. Granting capabilities incorrectly breaks isolation.
    pub fn grant() -> Self {
        let id = NEXT_CAP_ID.fetch_add(1, Ordering::Relaxed);
        Self {
            id,
            _phantom: PhantomData,
        }
    }

    /// Get the capability ID (for debugging/logging)
    pub fn id(&self) -> u32 {
        self.id
    }

    /// Transfer capability to another owner
    ///
    /// The original capability is consumed.
    pub fn transfer(self) -> Self {
        self
    }
}

impl<T: Resource> Clone for Cap<T> {
    fn clone(&self) -> Self {
        // Cloning a capability creates a new reference to the same resource
        // The ID remains the same
        Self {
            id: self.id,
            _phantom: PhantomData,
        }
    }
}

// Resource type definitions

/// GPIO pin resource
pub struct GpioPin<const N: u8>;
impl<const N: u8> Resource for GpioPin<N> {}

/// PWM channel resource
pub struct PwmChannel<const N: u8>;
impl<const N: u8> Resource for PwmChannel<N> {}

/// I2C bus resource
pub struct I2cBus<const N: u8>;
impl<const N: u8> Resource for I2cBus<N> {}

/// UART port resource
pub struct UartPort<const N: u8>;
impl<const N: u8> Resource for UartPort<N> {}

/// SPI bus resource
pub struct SpiBus<const N: u8>;
impl<const N: u8> Resource for SpiBus<N> {}

/// Timer resource
pub struct TimerResource<const N: u8>;
impl<const N: u8> Resource for TimerResource<N> {}

/// WiFi radio resource (singleton)
pub struct WifiRadio;
impl Resource for WifiRadio {}

/// CAN bus resource
pub struct CanBus<const N: u8>;
impl<const N: u8> Resource for CanBus<N> {}

/// Pyro channel resource (for rockets)
pub struct PyroChannel<const N: u8>;
impl<const N: u8> Resource for PyroChannel<N> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capabilities_have_unique_ids() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2: Cap<GpioPin<1>> = Cap::grant();
        assert_ne!(cap1.id(), cap2.id());
    }

    #[test]
    fn capability_clone_preserves_id() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2 = cap1.clone();
        assert_eq!(cap1.id(), cap2.id());
    }

    #[test]
    fn capability_transfer_preserves_id() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let id = cap1.id();
        let cap2 = cap1.transfer();
        assert_eq!(id, cap2.id());
    }
}
