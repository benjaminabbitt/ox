//! Ox Robotics Microkernel
//!
//! Main entry point for the ESP32-C3/C6/H2 robotics control kernel.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

// Kernel types - will be used as servers are implemented
#[allow(unused_imports)]
use ox_kernel::cap::Cap;
#[allow(unused_imports)]
use ox_kernel::ipc::Stream;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize ESP-HAL
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize Embassy time driver with Timer Group 0
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize logging
    esp_println::logger::init_logger_from_env();

    log::info!("================================");
    log::info!("  Ox Robotics Microkernel");
    log::info!("================================");
    log::info!("Chip: {}", ox_chip::CHIP.name);
    log::info!("Cores: {}", ox_chip::CHIP.cores);
    log::info!("Clock: {} MHz", ox_chip::CHIP.max_freq_mhz);

    // Spawn the heartbeat task
    spawner.spawn(heartbeat_task()).unwrap();

    log::info!("Kernel initialized, entering main loop");

    // Main supervisor loop
    loop {
        // TODO: Monitor server health, handle restarts
        embassy_time::Timer::after_millis(5000).await;
    }
}

/// Heartbeat task - blinks status and logs periodically
#[embassy_executor::task]
async fn heartbeat_task() {
    let mut counter: u32 = 0;
    loop {
        embassy_time::Timer::after_millis(1000).await;
        counter = counter.wrapping_add(1);
        log::info!("Heartbeat #{}", counter);
    }
}
