//! Procedural macros for ox-sketch
//!
//! Provides the `#[ox_sketch]` attribute macro that eliminates embassy boilerplate.

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// The main entry point macro for ox-sketch applications.
///
/// This macro transforms a simple async function into a complete embedded application
/// with all the necessary embassy and ESP-HAL setup.
///
/// # Example
///
/// ```ignore
/// #![no_std]
/// #![no_main]
///
/// use ox_sketch::prelude::*;
///
/// #[ox_sketch]
/// async fn main(mut bot: Robot) {
///     let mut led = bot.output(bot.pins.gpio2.take().unwrap());
///     loop {
///         led.toggle();
///         sleep(1000.millis()).await;
///     }
/// }
/// ```
///
/// This expands to embassy main, panic handler setup, timer initialization,
/// and Robot construction with peripheral access.
#[proc_macro_attribute]
pub fn ox_sketch(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);

    // Extract the user's function body
    let user_body = &input_fn.block;

    // Check that the function is async
    if input_fn.sig.asyncness.is_none() {
        return syn::Error::new_spanned(&input_fn.sig, "ox_sketch function must be async")
            .to_compile_error()
            .into();
    }

    // Generate the expanded code
    let expanded = quote! {
        // Provide defmt timestamp
        ::ox_sketch::__internal::defmt::timestamp!("{=u64:us}", ::ox_sketch::__internal::embassy_time::Instant::now().as_micros());

        #[::ox_sketch::__internal::esp_hal_embassy::main]
        async fn main(_spawner: ::ox_sketch::__internal::embassy_executor::Spawner) {
            // Initialize ESP-HAL
            let mut peripherals = ::ox_sketch::__internal::esp_hal::init(
                ::ox_sketch::__internal::esp_hal::Config::default()
            );

            // Take TIMG0 before creating Robot (Robot doesn't need it)
            let timg0 = core::mem::replace(
                &mut peripherals.TIMG0,
                unsafe { ::ox_sketch::__internal::esp_hal::peripherals::TIMG0::steal() }
            );

            // Initialize Embassy time driver
            let timg0 = ::ox_sketch::__internal::esp_hal::timer::timg::TimerGroup::new(timg0);
            ::ox_sketch::__internal::esp_hal_embassy::init(timg0.timer0);

            ::ox_sketch::__internal::defmt::info!("ox-sketch starting...");

            // Create the Robot with peripheral access
            let mut bot = ::ox_sketch::Robot::new(peripherals);

            // Run the user's code
            #user_body
        }
    };

    expanded.into()
}
