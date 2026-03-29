//! GPIO handles for simple pin control
//!
//! Provides OutputHandle and InputHandle for controlling GPIO pins
//! without dealing with ESP-HAL complexity.

use embassy_time::{Duration, Timer};
use esp_hal::gpio::{GpioPin, Input, Level, Output, Pull};

/// Handle to a GPIO output pin
///
/// # Example
/// ```ignore
/// let led = bot.led(); // or bot.output(pin)
/// led.high();
/// led.low();
/// led.toggle();
/// ```
pub struct OutputHandle {
    pin: Output<'static>,
}

impl OutputHandle {
    /// Create a new output handle from an ESP-HAL Output
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }

    /// Set the pin high (3.3V)
    pub fn high(&mut self) {
        self.pin.set_high();
    }

    /// Set the pin low (0V)
    pub fn low(&mut self) {
        self.pin.set_low();
    }

    /// Toggle the pin state
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }

    /// Set the pin to a specific state
    pub fn set(&mut self, on: bool) {
        if on {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
    }

    /// Check if pin is currently high
    pub fn is_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Check if pin is currently low
    pub fn is_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// Blink the LED with given on/off duration
    pub async fn blink(&mut self, on_ms: u64, off_ms: u64) {
        self.high();
        Timer::after(Duration::from_millis(on_ms)).await;
        self.low();
        Timer::after(Duration::from_millis(off_ms)).await;
    }
}

/// Handle to a GPIO input pin
///
/// # Example
/// ```ignore
/// let button = bot.button(); // or bot.input(pin)
/// if button.is_pressed() {
///     // do something
/// }
/// button.wait_for_press().await;
/// ```
pub struct InputHandle {
    pin: Input<'static>,
    active_low: bool,
}

impl InputHandle {
    /// Create a new input handle from an ESP-HAL Input
    pub fn new(pin: Input<'static>) -> Self {
        Self {
            pin,
            active_low: true, // Most buttons are active-low
        }
    }

    /// Create a new input handle with explicit active-high/low setting
    pub fn new_with_polarity(pin: Input<'static>, active_low: bool) -> Self {
        Self { pin, active_low }
    }

    /// Check if the input is high
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Check if the input is low
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Check if the button/input is pressed (accounts for active-low)
    pub fn is_pressed(&self) -> bool {
        if self.active_low {
            self.pin.is_low()
        } else {
            self.pin.is_high()
        }
    }

    /// Wait for the pin to go high
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    /// Wait for the pin to go low
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    /// Wait for button press (accounts for active-low)
    pub async fn wait_for_press(&mut self) {
        if self.active_low {
            self.pin.wait_for_low().await;
        } else {
            self.pin.wait_for_high().await;
        }
    }

    /// Wait for button release (accounts for active-low)
    pub async fn wait_for_release(&mut self) {
        if self.active_low {
            self.pin.wait_for_high().await;
        } else {
            self.pin.wait_for_low().await;
        }
    }

    /// Wait for press with debounce
    pub async fn wait_for_press_debounced(&mut self, debounce_ms: u64) {
        self.wait_for_press().await;
        Timer::after(Duration::from_millis(debounce_ms)).await;
        // Verify still pressed after debounce
        while !self.is_pressed() {
            self.wait_for_press().await;
            Timer::after(Duration::from_millis(debounce_ms)).await;
        }
    }
}

/// Helper trait for creating outputs from any GPIO pin
pub trait IntoOutput {
    fn into_output(self) -> Output<'static>;
}

/// Helper trait for creating inputs from any GPIO pin
pub trait IntoInput {
    fn into_input(self) -> Input<'static>;
    fn into_input_pullup(self) -> Input<'static>;
    fn into_input_pulldown(self) -> Input<'static>;
}

// Macro to generate IntoOutput/IntoInput impls for concrete pin numbers
macro_rules! impl_gpio_traits_for_pin {
    ($n:literal) => {
        impl IntoOutput for GpioPin<$n> {
            fn into_output(self) -> Output<'static> {
                Output::new(self, Level::Low)
            }
        }

        impl IntoInput for GpioPin<$n> {
            fn into_input(self) -> Input<'static> {
                Input::new(self, Pull::None)
            }

            fn into_input_pullup(self) -> Input<'static> {
                Input::new(self, Pull::Up)
            }

            fn into_input_pulldown(self) -> Input<'static> {
                Input::new(self, Pull::Down)
            }
        }
    };
}

// ESP32-C3 has GPIO 0-21 (some reserved for flash)
#[cfg(feature = "chip-esp32c3")]
mod chip_impls {
    use super::*;

    impl_gpio_traits_for_pin!(0);
    impl_gpio_traits_for_pin!(1);
    impl_gpio_traits_for_pin!(2);
    impl_gpio_traits_for_pin!(3);
    impl_gpio_traits_for_pin!(4);
    impl_gpio_traits_for_pin!(5);
    impl_gpio_traits_for_pin!(6);
    impl_gpio_traits_for_pin!(7);
    impl_gpio_traits_for_pin!(8);
    impl_gpio_traits_for_pin!(9);
    impl_gpio_traits_for_pin!(10);
    impl_gpio_traits_for_pin!(18);
    impl_gpio_traits_for_pin!(19);
    impl_gpio_traits_for_pin!(20);
    impl_gpio_traits_for_pin!(21);
}

// ESP32-C6 has more GPIO pins
#[cfg(feature = "chip-esp32c6")]
mod chip_impls {
    use super::*;

    impl_gpio_traits_for_pin!(0);
    impl_gpio_traits_for_pin!(1);
    impl_gpio_traits_for_pin!(2);
    impl_gpio_traits_for_pin!(3);
    impl_gpio_traits_for_pin!(4);
    impl_gpio_traits_for_pin!(5);
    impl_gpio_traits_for_pin!(6);
    impl_gpio_traits_for_pin!(7);
    impl_gpio_traits_for_pin!(8);
    impl_gpio_traits_for_pin!(9);
    impl_gpio_traits_for_pin!(10);
    impl_gpio_traits_for_pin!(11);
    impl_gpio_traits_for_pin!(12);
    impl_gpio_traits_for_pin!(13);
    impl_gpio_traits_for_pin!(14);
    impl_gpio_traits_for_pin!(15);
    impl_gpio_traits_for_pin!(16);
    impl_gpio_traits_for_pin!(17);
    impl_gpio_traits_for_pin!(18);
    impl_gpio_traits_for_pin!(19);
    impl_gpio_traits_for_pin!(20);
    impl_gpio_traits_for_pin!(21);
    impl_gpio_traits_for_pin!(22);
    impl_gpio_traits_for_pin!(23);
}

// ESP32-S3 has GPIO 0-48
#[cfg(feature = "chip-esp32s3")]
mod chip_impls {
    use super::*;

    impl_gpio_traits_for_pin!(0);
    impl_gpio_traits_for_pin!(1);
    impl_gpio_traits_for_pin!(2);
    impl_gpio_traits_for_pin!(3);
    impl_gpio_traits_for_pin!(4);
    impl_gpio_traits_for_pin!(5);
    impl_gpio_traits_for_pin!(6);
    impl_gpio_traits_for_pin!(7);
    impl_gpio_traits_for_pin!(8);
    impl_gpio_traits_for_pin!(9);
    impl_gpio_traits_for_pin!(10);
    impl_gpio_traits_for_pin!(11);
    impl_gpio_traits_for_pin!(12);
    impl_gpio_traits_for_pin!(13);
    impl_gpio_traits_for_pin!(14);
    impl_gpio_traits_for_pin!(15);
    impl_gpio_traits_for_pin!(16);
    impl_gpio_traits_for_pin!(17);
    impl_gpio_traits_for_pin!(18);
    impl_gpio_traits_for_pin!(19);
    impl_gpio_traits_for_pin!(20);
    impl_gpio_traits_for_pin!(21);
    impl_gpio_traits_for_pin!(35);
    impl_gpio_traits_for_pin!(36);
    impl_gpio_traits_for_pin!(37);
    impl_gpio_traits_for_pin!(38);
    impl_gpio_traits_for_pin!(39);
    impl_gpio_traits_for_pin!(40);
    impl_gpio_traits_for_pin!(41);
    impl_gpio_traits_for_pin!(42);
    impl_gpio_traits_for_pin!(43);
    impl_gpio_traits_for_pin!(44);
    impl_gpio_traits_for_pin!(45);
    impl_gpio_traits_for_pin!(46);
    impl_gpio_traits_for_pin!(47);
    impl_gpio_traits_for_pin!(48);
}
