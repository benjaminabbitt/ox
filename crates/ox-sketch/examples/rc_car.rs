//! RC Car example - control a differential drive robot with an RC transmitter
//!
//! This example reads CRSF input from an ExpressLRS/Crossfire receiver
//! and controls two motors for differential (tank) steering.
//!
//! Wiring:
//! - GPIO20: CRSF RX (receiver TX -> ESP RX)
//! - GPIO21: CRSF TX (not used for receive-only)
//! - GPIO4: Left motor forward
//! - GPIO5: Left motor reverse
//! - GPIO6: Right motor forward
//! - GPIO7: Right motor reverse
//!
//! To run: `cargo run --example rc_car --features "chip-esp32c3,rc,motor"`

#![no_std]
#![no_main]

use ox_sketch::prelude::*;

// CRSF baud rate
const CRSF_BAUD: u32 = 420_000;

#[ox_sketch]
async fn main(mut bot: Robot) {
    defmt::info!("RC Car starting...");

    // ========== Setup CRSF Receiver ==========
    let uart1 = bot.uart1();
    let rx_pin = bot.gpio20();
    let tx_pin = bot.gpio21();

    let uart_config = esp_hal::uart::Config::default().with_baudrate(CRSF_BAUD);

    let uart = esp_hal::uart::Uart::new(uart1, uart_config)
        .unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin)
        .into_async();

    let (rx, _tx) = uart.split();
    let mut rc = RcHandle::new_crsf(rx);

    defmt::info!("CRSF receiver on UART1 (GPIO20)");

    // ========== Setup Motors ==========
    // Take all pins first (mutable borrows)
    let lf_pin = bot.gpio4();
    let lr_pin = bot.gpio5();
    let rf_pin = bot.gpio6();
    let rr_pin = bot.gpio7();

    // Then create outputs
    let left_fwd = bot.output(lf_pin);
    let left_rev = bot.output(lr_pin);
    let right_fwd = bot.output(rf_pin);
    let right_rev = bot.output(rr_pin);

    let mut drive = DifferentialDrive::new(left_fwd, left_rev, right_fwd, right_rev);

    defmt::info!("Motors on GPIO 4-7");
    defmt::info!("Waiting for RC signal...");

    // ========== Main Control Loop ==========
    loop {
        // Read RC input (blocks until frame received)
        let input = rc.read().await;

        // Only control motors when armed and not in failsafe
        if input.armed && !input.failsafe {
            drive.arcade(input.throttle, input.steering);
        } else {
            drive.stop();

            if input.failsafe {
                defmt::warn!("FAILSAFE - motors stopped");
            }
        }
    }
}
