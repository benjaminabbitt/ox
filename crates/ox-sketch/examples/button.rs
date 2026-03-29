//! Button example - toggle LED on button press
//!
//! This example toggles the LED each time the button is pressed.
//! Demonstrates async input waiting with debouncing.
//!
//! Wiring:
//! - GPIO2: LED (active high)
//! - GPIO9: Button (connects to GND when pressed)
//!
//! To run: `cargo run --example button --features chip-esp32c3`

#![no_std]
#![no_main]

use ox_sketch::prelude::*;

#[ox_sketch]
async fn main(mut bot: Robot) {
    // Get the LED and button pins
    let led_pin = bot.gpio2();
    let button_pin = bot.gpio9();

    let mut led = bot.output(led_pin);
    let mut button = bot.input_pullup(button_pin);

    defmt::info!("Press button on GPIO9 to toggle LED");

    loop {
        // Wait for button press with 50ms debounce
        button.wait_for_press_debounced(50).await;
        defmt::info!("Button pressed!");

        // Toggle the LED
        led.toggle();

        // Wait for button release before allowing another press
        button.wait_for_release().await;
    }
}
