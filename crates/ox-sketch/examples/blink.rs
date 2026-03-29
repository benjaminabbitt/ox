//! Blink example - the "Hello World" of embedded systems
//!
//! This example blinks the onboard LED on GPIO2.
//!
//! To run: `cargo run --example blink --features chip-esp32c3`

#![no_std]
#![no_main]

use ox_sketch::prelude::*;

#[ox_sketch]
async fn main(mut bot: Robot) {
    // Get the LED pin (GPIO2 on most ESP32-C3 dev boards)
    let pin = bot.gpio2();
    let mut led = bot.output(pin);

    defmt::info!("Blinking LED on GPIO2");

    loop {
        led.high();
        sleep(500.millis()).await;

        led.low();
        sleep(500.millis()).await;
    }
}
