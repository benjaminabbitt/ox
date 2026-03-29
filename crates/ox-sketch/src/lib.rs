//! # ox-sketch: Arduino-like simplicity for Rust robotics
//!
//! ox-sketch provides a beginner-friendly API for programming ESP32 robots.
//! It eliminates boilerplate and provides simple, intuitive interfaces.
//!
//! ## Quick Start
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use ox_sketch::prelude::*;
//!
//! #[ox_sketch]
//! async fn main(mut bot: Robot) {
//!     let mut led = bot.output(bot.gpio2());
//!     loop {
//!         led.toggle();
//!         sleep(1000.millis()).await;
//!     }
//! }
//! ```

#![no_std]

mod gpio;

#[cfg(feature = "rc")]
pub mod rc;

#[cfg(feature = "motor")]
pub mod motor;

#[cfg(feature = "pwm")]
pub mod pwm;

pub use gpio::{InputHandle, IntoInput, IntoOutput, OutputHandle};
pub use ox_sketch_macros::ox_sketch;

// Re-export embassy_time for sleep functions
pub use embassy_time::{Duration, Timer};

/// Prelude module - import everything you need with `use ox_sketch::prelude::*`
pub mod prelude {
    pub use crate::gpio::{InputHandle, IntoInput, IntoOutput, OutputHandle};
    pub use crate::ox_sketch;
    pub use crate::Robot;
    pub use crate::{sleep, DurationExt};
    pub use defmt;
    pub use embassy_time::Duration;

    // RC types (when rc feature enabled)
    #[cfg(feature = "rc")]
    pub use crate::rc::{RcHandle, RcInput};

    // Motor types (when motor feature enabled)
    #[cfg(feature = "motor")]
    pub use crate::motor::{DifferentialDrive, Motor};

    // PWM types (when pwm feature enabled)
    #[cfg(feature = "pwm")]
    pub use crate::pwm::{CarDrive, PwmDifferentialDrive, PwmMotor, Servo};
}

/// Internal module for macro-generated code. Not public API.
#[doc(hidden)]
pub mod __internal {
    pub use defmt;
    pub use embassy_executor;
    pub use embassy_time;
    pub use esp_hal;
    pub use esp_hal_embassy;

    // These are used by the macro-generated code for panic handling
    #[allow(unused_imports)]
    pub use esp_backtrace as _;
    #[allow(unused_imports)]
    pub use esp_println as _;
}

// Re-export LEDC types for PWM setup
#[cfg(feature = "pwm")]
pub mod ledc {
    //! LEDC (PWM) peripheral types
    //!
    //! Re-exports from esp-hal for configuring PWM timers and channels.
    pub use esp_hal::ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
        Ledc, LowSpeed, LSGlobalClkSource,
    };

    // Re-export fugit for frequency extensions
    pub use fugit::RateExtU32;
}

/// Extension trait for creating durations with friendly syntax
pub trait DurationExt {
    fn millis(self) -> Duration;
    fn secs(self) -> Duration;
    fn hz(self) -> Duration;
}

impl DurationExt for u64 {
    fn millis(self) -> Duration {
        Duration::from_millis(self)
    }

    fn secs(self) -> Duration {
        Duration::from_secs(self)
    }

    fn hz(self) -> Duration {
        Duration::from_hz(self)
    }
}

/// Sleep for a duration. Friendly wrapper around Timer::after.
pub async fn sleep(duration: Duration) {
    Timer::after(duration).await;
}

/// The main Robot struct that provides access to all hardware.
///
/// You receive this in your `#[ox_sketch]` function and use it to
/// access GPIO pins, motors, sensors, etc.
///
/// # Example
/// ```ignore
/// #[ox_sketch]
/// async fn main(mut bot: Robot) {
///     // Create an output on GPIO2
///     let mut led = bot.output(bot.gpio2());
///
///     // Create an input with pullup on GPIO9
///     let mut button = bot.input_pullup(bot.gpio9());
///
///     loop {
///         button.wait_for_press().await;
///         led.toggle();
///     }
/// }
/// ```
pub struct Robot {
    peripherals: esp_hal::peripherals::Peripherals,
}

impl Robot {
    /// Create a new Robot with the given peripherals.
    /// This is called automatically by the `#[ox_sketch]` macro.
    #[doc(hidden)]
    pub fn new(peripherals: esp_hal::peripherals::Peripherals) -> Self {
        Self { peripherals }
    }

    /// Create an output pin
    ///
    /// # Example
    /// ```ignore
    /// let led = bot.output(bot.gpio2());
    /// led.high();
    /// ```
    pub fn output<P: IntoOutput>(&self, pin: P) -> OutputHandle {
        OutputHandle::new(pin.into_output())
    }

    /// Create an input pin with internal pull-up resistor
    ///
    /// Most buttons should use this - they connect the pin to ground when pressed.
    pub fn input_pullup<P: IntoInput>(&self, pin: P) -> InputHandle {
        InputHandle::new(pin.into_input_pullup())
    }

    /// Create an input pin with internal pull-down resistor
    pub fn input_pulldown<P: IntoInput>(&self, pin: P) -> InputHandle {
        InputHandle::new_with_polarity(pin.into_input_pulldown(), false)
    }

    /// Create a floating input pin (no pull resistor)
    pub fn input<P: IntoInput>(&self, pin: P) -> InputHandle {
        InputHandle::new_with_polarity(pin.into_input(), false)
    }
}

// ESP32-C3 GPIO pins and peripherals
#[cfg(feature = "chip-esp32c3")]
mod chip_pins {
    use super::*;

    // Macro to generate gpio accessor methods
    macro_rules! gpio_method {
        ($name:ident, $num:literal, $field:ident, $doc:expr) => {
            impl Robot {
                #[doc = $doc]
                pub fn $name(&mut self) -> esp_hal::gpio::GpioPin<$num> {
                    core::mem::replace(&mut self.peripherals.$field, unsafe {
                        esp_hal::gpio::GpioPin::steal()
                    })
                }
            }
        };
    }

    gpio_method!(gpio0, 0, GPIO0, "Take GPIO0");
    gpio_method!(gpio1, 1, GPIO1, "Take GPIO1");
    gpio_method!(gpio2, 2, GPIO2, "Take GPIO2 (commonly used for onboard LED)");
    gpio_method!(gpio3, 3, GPIO3, "Take GPIO3");
    gpio_method!(gpio4, 4, GPIO4, "Take GPIO4");
    gpio_method!(gpio5, 5, GPIO5, "Take GPIO5");
    gpio_method!(gpio6, 6, GPIO6, "Take GPIO6");
    gpio_method!(gpio7, 7, GPIO7, "Take GPIO7");
    gpio_method!(gpio8, 8, GPIO8, "Take GPIO8");
    gpio_method!(gpio9, 9, GPIO9, "Take GPIO9 (commonly used for boot button)");
    gpio_method!(gpio10, 10, GPIO10, "Take GPIO10");
    gpio_method!(gpio18, 18, GPIO18, "Take GPIO18");
    gpio_method!(gpio19, 19, GPIO19, "Take GPIO19");
    gpio_method!(gpio20, 20, GPIO20, "Take GPIO20");
    gpio_method!(gpio21, 21, GPIO21, "Take GPIO21");

    // UART peripherals
    impl Robot {
        /// Take UART0 peripheral
        pub fn uart0(&mut self) -> esp_hal::peripherals::UART0 {
            core::mem::replace(&mut self.peripherals.UART0, unsafe {
                esp_hal::peripherals::UART0::steal()
            })
        }

        /// Take UART1 peripheral
        pub fn uart1(&mut self) -> esp_hal::peripherals::UART1 {
            core::mem::replace(&mut self.peripherals.UART1, unsafe {
                esp_hal::peripherals::UART1::steal()
            })
        }

        /// Take LEDC (PWM) peripheral
        pub fn ledc(&mut self) -> esp_hal::peripherals::LEDC {
            core::mem::replace(&mut self.peripherals.LEDC, unsafe {
                esp_hal::peripherals::LEDC::steal()
            })
        }
    }
}

// ESP32-C6 GPIO pins
#[cfg(feature = "chip-esp32c6")]
mod chip_pins {
    use super::*;

    macro_rules! gpio_method {
        ($name:ident, $num:literal, $field:ident, $doc:expr) => {
            impl Robot {
                #[doc = $doc]
                pub fn $name(&mut self) -> esp_hal::gpio::GpioPin<$num> {
                    core::mem::replace(&mut self.peripherals.$field, unsafe {
                        esp_hal::gpio::GpioPin::steal()
                    })
                }
            }
        };
    }

    gpio_method!(gpio2, 2, GPIO2, "Take GPIO2");
    gpio_method!(gpio9, 9, GPIO9, "Take GPIO9");
    // Add more C6 pins as needed
}

// ESP32-S3 GPIO pins
#[cfg(feature = "chip-esp32s3")]
mod chip_pins {
    use super::*;

    macro_rules! gpio_method {
        ($name:ident, $num:literal, $field:ident, $doc:expr) => {
            impl Robot {
                #[doc = $doc]
                pub fn $name(&mut self) -> esp_hal::gpio::GpioPin<$num> {
                    core::mem::replace(&mut self.peripherals.$field, unsafe {
                        esp_hal::gpio::GpioPin::steal()
                    })
                }
            }
        };
    }

    gpio_method!(gpio2, 2, GPIO2, "Take GPIO2");
    gpio_method!(gpio9, 9, GPIO9, "Take GPIO9");
    // Add more S3 pins as needed
}
