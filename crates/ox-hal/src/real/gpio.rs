//! Real GPIO implementation for ESP32
//!
//! Wraps esp-hal GPIO pins.

use crate::gpio::{InputPin, OutputPin};
use super::HalError;

use esp_hal::gpio::{Input, Output, Level, Pull};
use esp_hal::peripheral::Peripheral;

/// Wrapper for esp-hal output pin
pub struct EspOutputPin<'d> {
    pin: Output<'d>,
}

impl<'d> EspOutputPin<'d> {
    /// Create from an esp-hal Output pin
    pub fn new(pin: Output<'d>) -> Self {
        Self { pin }
    }

    /// Create output pin from a peripheral pin
    pub fn from_peripheral<P>(pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: esp_hal::gpio::OutputPin,
    {
        Self {
            pin: Output::new(pin, Level::Low),
        }
    }
}

impl OutputPin for EspOutputPin<'_> {
    type Error = HalError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.pin.toggle();
        Ok(())
    }

    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_set_high())
    }
}

/// Wrapper for esp-hal input pin
pub struct EspInputPin<'d> {
    pin: Input<'d>,
}

impl<'d> EspInputPin<'d> {
    /// Create from an esp-hal Input pin
    pub fn new(pin: Input<'d>) -> Self {
        Self { pin }
    }

    /// Create input pin from a peripheral pin (no pull)
    pub fn from_peripheral<P>(pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: esp_hal::gpio::InputPin,
    {
        Self {
            pin: Input::new(pin, Pull::None),
        }
    }

    /// Create input pin with pull-up
    pub fn from_peripheral_pullup<P>(pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: esp_hal::gpio::InputPin,
    {
        Self {
            pin: Input::new(pin, Pull::Up),
        }
    }

    /// Create input pin with pull-down
    pub fn from_peripheral_pulldown<P>(pin: impl Peripheral<P = P> + 'd) -> Self
    where
        P: esp_hal::gpio::InputPin,
    {
        Self {
            pin: Input::new(pin, Pull::Down),
        }
    }
}

impl InputPin for EspInputPin<'_> {
    type Error = HalError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_low())
    }
}
