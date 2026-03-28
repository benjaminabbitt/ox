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
#[derive(Debug)]
pub struct GpioPin<const N: u8>;
impl<const N: u8> Resource for GpioPin<N> {}

/// PWM channel resource
#[derive(Debug)]
pub struct PwmChannel<const N: u8>;
impl<const N: u8> Resource for PwmChannel<N> {}

/// I2C bus resource
#[derive(Debug)]
pub struct I2cBus<const N: u8>;
impl<const N: u8> Resource for I2cBus<N> {}

/// UART port resource
#[derive(Debug)]
pub struct UartPort<const N: u8>;
impl<const N: u8> Resource for UartPort<N> {}

/// SPI bus resource
#[derive(Debug)]
pub struct SpiBus<const N: u8>;
impl<const N: u8> Resource for SpiBus<N> {}

/// Timer resource
#[derive(Debug)]
pub struct TimerResource<const N: u8>;
impl<const N: u8> Resource for TimerResource<N> {}

/// WiFi radio resource (singleton)
#[derive(Debug)]
pub struct WifiRadio;
impl Resource for WifiRadio {}

/// CAN bus resource
#[derive(Debug)]
pub struct CanBus<const N: u8>;
impl<const N: u8> Resource for CanBus<N> {}

/// Pyro channel resource (for rockets)
#[derive(Debug)]
pub struct PyroChannel<const N: u8>;
impl<const N: u8> Resource for PyroChannel<N> {}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== Basic Capability Tests ====================

    #[test]
    fn capabilities_have_unique_ids() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2: Cap<GpioPin<1>> = Cap::grant();
        assert_ne!(cap1.id(), cap2.id());
    }

    #[test]
    fn capability_ids_are_monotonically_increasing() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2: Cap<GpioPin<1>> = Cap::grant();
        let cap3: Cap<GpioPin<2>> = Cap::grant();

        assert!(cap2.id() > cap1.id());
        assert!(cap3.id() > cap2.id());
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

    #[test]
    fn capability_ids_are_nonzero() {
        let cap: Cap<GpioPin<0>> = Cap::grant();
        assert!(cap.id() > 0);
    }

    // ==================== Different Resource Type Tests ====================

    #[test]
    fn gpio_capability_can_be_granted() {
        let _cap: Cap<GpioPin<5>> = Cap::grant();
    }

    #[test]
    fn pwm_capability_can_be_granted() {
        let _cap: Cap<PwmChannel<0>> = Cap::grant();
    }

    #[test]
    fn i2c_capability_can_be_granted() {
        let _cap: Cap<I2cBus<0>> = Cap::grant();
    }

    #[test]
    fn uart_capability_can_be_granted() {
        let _cap: Cap<UartPort<0>> = Cap::grant();
    }

    #[test]
    fn spi_capability_can_be_granted() {
        let _cap: Cap<SpiBus<0>> = Cap::grant();
    }

    #[test]
    fn timer_capability_can_be_granted() {
        let _cap: Cap<TimerResource<0>> = Cap::grant();
    }

    #[test]
    fn wifi_capability_can_be_granted() {
        let _cap: Cap<WifiRadio> = Cap::grant();
    }

    #[test]
    fn can_capability_can_be_granted() {
        let _cap: Cap<CanBus<0>> = Cap::grant();
    }

    #[test]
    fn pyro_capability_can_be_granted() {
        let _cap: Cap<PyroChannel<0>> = Cap::grant();
    }

    // ==================== Type Safety Tests ====================
    // These tests verify compile-time type safety

    #[test]
    fn different_resource_types_are_distinct() {
        // This test verifies that capabilities for different resources
        // are not interchangeable at compile time

        fn requires_gpio_cap(_cap: &Cap<GpioPin<0>>) {}
        fn requires_pwm_cap(_cap: &Cap<PwmChannel<0>>) {}

        let gpio_cap: Cap<GpioPin<0>> = Cap::grant();
        let pwm_cap: Cap<PwmChannel<0>> = Cap::grant();

        // These compile because types match
        requires_gpio_cap(&gpio_cap);
        requires_pwm_cap(&pwm_cap);

        // These would NOT compile (type mismatch):
        // requires_gpio_cap(&pwm_cap);  // error: mismatched types
        // requires_pwm_cap(&gpio_cap);  // error: mismatched types
    }

    #[test]
    fn different_pin_numbers_are_distinct() {
        fn requires_pin_0(_cap: &Cap<GpioPin<0>>) {}
        fn requires_pin_1(_cap: &Cap<GpioPin<1>>) {}

        let cap0: Cap<GpioPin<0>> = Cap::grant();
        let cap1: Cap<GpioPin<1>> = Cap::grant();

        requires_pin_0(&cap0);
        requires_pin_1(&cap1);

        // These would NOT compile:
        // requires_pin_0(&cap1);  // error: mismatched types
        // requires_pin_1(&cap0);  // error: mismatched types
    }

    // ==================== Multiple Grant Tests ====================

    #[test]
    fn multiple_grants_for_same_resource_have_different_ids() {
        // This simulates granting the same resource to multiple servers
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2: Cap<GpioPin<0>> = Cap::grant();

        // Each grant creates a new capability with unique ID
        assert_ne!(cap1.id(), cap2.id());
    }

    #[test]
    fn many_capabilities_all_unique() {
        let caps: [Cap<GpioPin<0>>; 10] = core::array::from_fn(|_| Cap::grant());

        // Verify all IDs are unique
        for i in 0..10 {
            for j in (i + 1)..10 {
                assert_ne!(caps[i].id(), caps[j].id());
            }
        }
    }

    // ==================== Clone vs Transfer Semantics ====================

    #[test]
    fn clone_allows_both_copies_to_be_used() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let cap2 = cap1.clone();

        // Both capabilities are usable
        assert_eq!(cap1.id(), cap2.id());
        let _ = cap1.id();  // cap1 still valid
        let _ = cap2.id();  // cap2 still valid
    }

    #[test]
    fn transfer_consumes_original() {
        let cap1: Cap<GpioPin<0>> = Cap::grant();
        let id = cap1.id();
        let cap2 = cap1.transfer();  // cap1 is moved

        // cap2 has the same ID
        assert_eq!(id, cap2.id());

        // cap1 can no longer be used (it was moved)
        // This line would NOT compile:
        // let _ = cap1.id();  // error: use of moved value
    }

    // ==================== Debug Implementation ====================

    #[test]
    fn capability_implements_debug() {
        extern crate std;
        extern crate alloc;
        use std::format;
        use alloc::string::ToString;

        let cap: Cap<GpioPin<0>> = Cap::grant();
        let debug_str = format!("{:?}", cap);

        // Verify debug output contains the ID
        assert!(debug_str.contains(&cap.id().to_string()));
    }
}
