//! Ox Robotics Microkernel - HAL Test Firmware
//!
//! Tests GPIO and I2C with simulated hardware in Wokwi.
//!
//! Wiring (diagram.json):
//! - GPIO 2: Green LED (output)
//! - GPIO 3: Red LED (output)
//! - GPIO 9: Button (input, active low)
//! - GPIO 4: I2C SDA (MPU6050)
//! - GPIO 5: I2C SCL (MPU6050)
//! - GPIO 6: Servo PWM (simulated via fast GPIO toggle)

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;

// Kernel types
#[allow(unused_imports)]
use ox_kernel::cap::Cap;
#[allow(unused_imports)]
use ox_kernel::ipc::Stream;

// MPU6050 I2C address and registers
// Note: I2C async blocks in Wokwi simulation - works on real hardware
#[allow(dead_code)]
const MPU6050_ADDR: u8 = 0x68;
#[allow(dead_code)]
const MPU6050_WHO_AM_I: u8 = 0x75;
#[allow(dead_code)]
const MPU6050_PWR_MGMT_1: u8 = 0x6B;
#[allow(dead_code)]
const MPU6050_ACCEL_XOUT_H: u8 = 0x3B;

// Signal for button press events
static BUTTON_PRESSED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize ESP-HAL
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Initialize Embassy time driver
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize logging
    esp_println::logger::init_logger_from_env();

    log::info!("================================");
    log::info!("  Ox HAL Test Firmware");
    log::info!("================================");
    log::info!("Chip: {}", ox_chip::CHIP.name);

    // ========== GPIO Setup ==========
    let led_green = Output::new(peripherals.GPIO2, Level::Low);
    let led_red = Output::new(peripherals.GPIO3, Level::Low);
    let button = Input::new(peripherals.GPIO9, Pull::Up);
    let servo_pin = Output::new(peripherals.GPIO6, Level::Low);

    log::info!("[GPIO] LEDs on GPIO2 (green), GPIO3 (red)");
    log::info!("[GPIO] Button on GPIO9 (pull-up)");
    log::info!("[GPIO] Servo signal on GPIO6");

    // ========== I2C Setup ==========
    let i2c_config = I2cConfig::default();
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO4)
        .with_scl(peripherals.GPIO5)
        .into_async();

    log::info!("[I2C] Initialized on GPIO4 (SDA), GPIO5 (SCL)");

    // ========== Spawn Test Tasks ==========
    spawner.spawn(gpio_test_task(led_green, led_red)).unwrap();
    spawner.spawn(button_monitor_task(button)).unwrap();
    spawner.spawn(servo_simulate_task(servo_pin)).unwrap();

    // I2C test disabled in Wokwi - async I2C blocks the executor.
    // Uncomment for real hardware testing:
    // spawner.spawn(i2c_test_task(i2c)).unwrap();
    let _ = i2c;

    log::info!("All HAL test tasks spawned");
    log::info!("================================");

    // Main loop - respond to button presses
    loop {
        BUTTON_PRESSED.wait().await;
        log::info!("[MAIN] Button press detected!");
    }
}

/// GPIO test - alternates LEDs every 500ms
#[embassy_executor::task]
async fn gpio_test_task(mut led_green: Output<'static>, mut led_red: Output<'static>) {
    log::info!("[GPIO] Starting LED blink test");

    let mut toggle = false;
    let mut count = 0u32;
    loop {
        if toggle {
            led_green.set_high();
            led_red.set_low();
        } else {
            led_green.set_low();
            led_red.set_high();
        }
        toggle = !toggle;
        count += 1;

        // Log every 4 toggles (2 seconds)
        if count % 4 == 0 {
            log::info!("[GPIO] LED toggle count: {}", count);
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Button monitor - signals main task on press
#[embassy_executor::task]
async fn button_monitor_task(button: Input<'static>) {
    log::info!("[GPIO] Monitoring button on GPIO9");

    let mut last_state = true; // Pull-up, so high when not pressed
    loop {
        let current = button.is_high();

        // Detect falling edge (button press)
        if last_state && !current {
            BUTTON_PRESSED.signal(());
        }
        last_state = current;

        Timer::after(Duration::from_millis(50)).await; // Debounce
    }
}

/// I2C test - reads MPU6050 WHO_AM_I and accelerometer data
/// Note: Disabled in Wokwi (async I2C blocks executor), works on real hardware
#[allow(dead_code)]
#[embassy_executor::task]
async fn i2c_test_task(mut i2c: I2c<'static, Async>) {
    log::info!("[I2C] Starting MPU6050 test");

    // Small delay for I2C to stabilize
    log::info!("[I2C] Waiting 100ms for I2C to stabilize...");
    Timer::after(Duration::from_millis(100)).await;
    log::info!("[I2C] Reading WHO_AM_I register...");

    // Read WHO_AM_I register with timeout
    let mut buf = [0u8; 1];
    match with_timeout(
        Duration::from_millis(500),
        i2c.write_read(MPU6050_ADDR, &[MPU6050_WHO_AM_I], &mut buf),
    )
    .await
    {
        Ok(Ok(())) => {
            log::info!("[I2C] MPU6050 WHO_AM_I: 0x{:02X} (expected 0x68)", buf[0]);
        }
        Ok(Err(e)) => {
            log::error!("[I2C] Failed to read WHO_AM_I: {:?}", e);
            return;
        }
        Err(_) => {
            log::error!("[I2C] WHO_AM_I read timed out - I2C bus may be stuck");
            return;
        }
    }

    // Wake up MPU6050 (clear sleep bit)
    match i2c.write(MPU6050_ADDR, &[MPU6050_PWR_MGMT_1, 0x00]).await {
        Ok(()) => log::info!("[I2C] MPU6050 woken up"),
        Err(e) => {
            log::error!("[I2C] Failed to wake MPU6050: {:?}", e);
            return;
        }
    }

    // Periodically read accelerometer data
    loop {
        Timer::after(Duration::from_millis(1000)).await;

        let mut accel_buf = [0u8; 6];
        match i2c
            .write_read(MPU6050_ADDR, &[MPU6050_ACCEL_XOUT_H], &mut accel_buf)
            .await
        {
            Ok(()) => {
                let ax = i16::from_be_bytes([accel_buf[0], accel_buf[1]]);
                let ay = i16::from_be_bytes([accel_buf[2], accel_buf[3]]);
                let az = i16::from_be_bytes([accel_buf[4], accel_buf[5]]);
                log::info!("[I2C] Accel: X={:6} Y={:6} Z={:6}", ax, ay, az);
            }
            Err(e) => {
                log::error!("[I2C] Accel read error: {:?}", e);
            }
        }
    }
}

/// Servo simulation via GPIO pulse width (software PWM)
///
/// Generates ~50Hz signal with varying pulse width to control servo position.
/// This demonstrates GPIO timing without requiring LEDC peripheral complexity.
#[embassy_executor::task]
async fn servo_simulate_task(mut pin: Output<'static>) {
    log::info!("[PWM] Starting servo simulation (software PWM)");

    // Servo pulse widths: 1ms (0°) to 2ms (180°) at 50Hz (20ms period)
    const PERIOD_US: u64 = 20_000; // 20ms = 50Hz
    const MIN_PULSE_US: u64 = 1000; // 1ms = 0 degrees
    const MAX_PULSE_US: u64 = 2000; // 2ms = 180 degrees
    const STEP_US: u64 = 50;

    let mut pulse_us = MIN_PULSE_US;
    let mut increasing = true;
    let mut cycle_count = 0u32;

    loop {
        // Generate one PWM cycle
        pin.set_high();
        Timer::after(Duration::from_micros(pulse_us)).await;
        pin.set_low();
        Timer::after(Duration::from_micros(PERIOD_US - pulse_us)).await;

        cycle_count += 1;

        // Update pulse width every 10 cycles (~200ms)
        if cycle_count % 10 == 0 {
            let angle = ((pulse_us - MIN_PULSE_US) * 180) / (MAX_PULSE_US - MIN_PULSE_US);
            log::info!("[PWM] Servo pulse: {}us (~{}°)", pulse_us, angle);

            if increasing {
                pulse_us += STEP_US;
                if pulse_us >= MAX_PULSE_US {
                    pulse_us = MAX_PULSE_US;
                    increasing = false;
                }
            } else {
                pulse_us -= STEP_US;
                if pulse_us <= MIN_PULSE_US {
                    pulse_us = MIN_PULSE_US;
                    increasing = true;
                }
            }
        }
    }
}
