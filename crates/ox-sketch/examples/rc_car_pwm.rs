//! RC Car with PWM speed control and servo steering
//!
//! This example reads CRSF input from an ExpressLRS/Crossfire receiver
//! and controls a car with PWM motor speed control and a steering servo.
//!
//! Wiring:
//! - GPIO20: CRSF RX (receiver TX -> ESP RX)
//! - GPIO21: CRSF TX (not used for receive-only)
//! - GPIO4: Steering servo signal
//! - GPIO5: Motor PWM (speed control)
//! - GPIO6: Motor direction
//!
//! To run: `cargo run --example rc_car_pwm --features "chip-esp32c3,rc,pwm"`

#![no_std]
#![no_main]

use ox_sketch::prelude::*;
use ox_sketch::ledc::{
    channel, timer, ChannelIFace, Ledc, LSGlobalClkSource, LowSpeed, RateExtU32, TimerIFace,
};

// CRSF baud rate
const CRSF_BAUD: u32 = 420_000;

#[ox_sketch]
async fn main(mut bot: Robot) {
    defmt::info!("RC Car (PWM) starting...");

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

    // ========== Setup LEDC for PWM ==========
    let ledc_peripheral = bot.ledc();
    let mut ledc = Ledc::new(ledc_peripheral);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Timer 0: 50Hz for servo
    let mut servo_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50.Hz(),
        })
        .unwrap();

    // Timer 1: 20kHz for motor (inaudible PWM frequency)
    let mut motor_timer = ledc.timer::<LowSpeed>(timer::Number::Timer1);
    motor_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 20.kHz(),
        })
        .unwrap();

    // ========== Setup Steering Servo ==========
    let servo_pin = bot.gpio4();
    let mut servo_channel = ledc.channel(channel::Number::Channel0, servo_pin);
    servo_channel
        .configure(channel::config::Config {
            timer: &servo_timer,
            duty_pct: 7, // Start centered (~1.5ms pulse)
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut steering = Servo::new(servo_channel);
    steering.center();

    defmt::info!("Steering servo on GPIO4");

    // ========== Setup Drive Motor ==========
    let motor_pwm_pin = bot.gpio5();
    let mut motor_channel = ledc.channel(channel::Number::Channel1, motor_pwm_pin);
    motor_channel
        .configure(channel::config::Config {
            timer: &motor_timer,
            duty_pct: 0, // Start stopped
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let motor_dir_pin = bot.gpio6();
    let motor_dir = bot.output(motor_dir_pin);

    let motor = PwmMotor::new(motor_channel, motor_dir);

    defmt::info!("Motor PWM on GPIO5, direction on GPIO6");

    // Create car drive controller
    let mut car = CarDrive::new(motor, steering);

    defmt::info!("Waiting for RC signal...");

    // ========== Main Control Loop ==========
    loop {
        // Read RC input (blocks until frame received)
        let input = rc.read().await;

        // Only control when armed and not in failsafe
        if input.armed && !input.failsafe {
            // Drive with throttle and steering from RC
            car.drive(input.throttle, input.steering);
        } else {
            car.stop();

            if input.failsafe {
                defmt::warn!("FAILSAFE - stopped");
            }
        }
    }
}
