//! PWM control for servos and motors
//!
//! Provides simple interfaces for PWM-controlled devices:
//! - `Servo`: RC servo control with angle positioning
//! - `PwmMotor`: DC motor with variable speed control
//!
//! Uses the ESP32 LEDC peripheral for PWM generation.

use crate::gpio::OutputHandle;
use esp_hal::ledc::{
    channel::{Channel, ChannelIFace},
    LowSpeed,
};

/// RC servo controller
///
/// Controls an RC-style servo using 50Hz PWM with 1-2ms pulse width.
/// Position ranges from -1.0 (full left) to 1.0 (full right).
///
/// # Example
/// ```ignore
/// // Setup LEDC for servo (50Hz)
/// let mut ledc = Ledc::new(bot.ledc());
/// ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
///
/// let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
/// timer.configure(timer::config::Config {
///     duty: timer::config::Duty::Duty10Bit,
///     clock_source: timer::LSClockSource::APBClk,
///     frequency: 50.Hz(),
/// }).unwrap();
///
/// let servo_pin = bot.gpio4();
/// let mut channel = ledc.channel(channel::Number::Channel0, servo_pin);
/// channel.configure(channel::config::Config {
///     timer: &timer,
///     duty_pct: 7,
///     pin_config: channel::config::PinConfig::PushPull,
/// }).unwrap();
///
/// let mut servo = Servo::new(channel);
/// servo.center();        // Move to center
/// servo.set(0.5);        // Turn right 50%
/// servo.set(-1.0);       // Full left
/// ```
pub struct Servo<'d> {
    channel: Channel<'d, LowSpeed>,
    /// Duty percentage for minimum position (-1.0), default 5% (~1ms)
    min_duty_pct: u8,
    /// Duty percentage for maximum position (1.0), default 10% (~2ms)
    max_duty_pct: u8,
}

impl<'d> Servo<'d> {
    /// Create a new servo from a configured LEDC channel
    ///
    /// The channel should be configured for 50Hz with at least 10-bit duty resolution.
    pub fn new(channel: Channel<'d, LowSpeed>) -> Self {
        Self {
            channel,
            min_duty_pct: 5,  // ~1ms at 50Hz = 0 degrees
            max_duty_pct: 10, // ~2ms at 50Hz = 180 degrees
        }
    }

    /// Create with custom pulse width range
    ///
    /// Standard servos use 5-10% duty at 50Hz (1-2ms pulse).
    /// Some servos have extended range (e.g., 2.5-12.5% for 0.5-2.5ms).
    pub fn with_range(channel: Channel<'d, LowSpeed>, min_duty_pct: u8, max_duty_pct: u8) -> Self {
        Self {
            channel,
            min_duty_pct,
            max_duty_pct,
        }
    }

    /// Set servo position
    ///
    /// - position: [-1.0, 1.0] where -1.0 = full left, 0.0 = center, 1.0 = full right
    pub fn set(&mut self, position: f32) {
        let position = position.clamp(-1.0, 1.0);
        // Map -1..1 to 0..1
        let normalized = (position + 1.0) / 2.0;
        let duty_range = (self.max_duty_pct - self.min_duty_pct) as f32;
        let duty = self.min_duty_pct + (normalized * duty_range) as u8;
        let _ = self.channel.set_duty(duty);
    }

    /// Set to center position (0.0)
    pub fn center(&mut self) {
        self.set(0.0);
    }

    /// Set raw duty percentage (0-100)
    pub fn set_duty(&mut self, duty_pct: u8) {
        let _ = self.channel.set_duty(duty_pct);
    }
}

/// PWM motor with variable speed and direction
///
/// Uses one PWM channel for speed control and one GPIO for direction.
/// This is suitable for motor drivers that have PWM + DIR inputs
/// (like many brushed DC motor drivers).
///
/// # Example
/// ```ignore
/// // Setup LEDC for motor (20kHz for quiet operation)
/// let mut ledc = Ledc::new(bot.ledc());
/// ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
///
/// let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer1);
/// timer.configure(timer::config::Config {
///     duty: timer::config::Duty::Duty8Bit,
///     clock_source: timer::LSClockSource::APBClk,
///     frequency: 20.kHz(),
/// }).unwrap();
///
/// let pwm_pin = bot.gpio5();
/// let mut channel = ledc.channel(channel::Number::Channel1, pwm_pin);
/// channel.configure(channel::config::Config {
///     timer: &timer,
///     duty_pct: 0,
///     pin_config: channel::config::PinConfig::PushPull,
/// }).unwrap();
///
/// let dir_pin = bot.gpio6();
/// let dir = bot.output(dir_pin);
///
/// let mut motor = PwmMotor::new(channel, dir);
/// motor.set(0.5);   // 50% forward
/// motor.set(-0.8);  // 80% reverse
/// motor.stop();
/// ```
pub struct PwmMotor<'d> {
    pwm: Channel<'d, LowSpeed>,
    dir: OutputHandle,
    deadzone: f32,
}

impl<'d> PwmMotor<'d> {
    /// Create a new PWM motor controller
    ///
    /// - pwm: LEDC channel for speed control (should be configured for 20kHz+)
    /// - dir: GPIO output for direction (high = forward, low = reverse)
    pub fn new(pwm: Channel<'d, LowSpeed>, dir: OutputHandle) -> Self {
        Self {
            pwm,
            dir,
            deadzone: 0.05,
        }
    }

    /// Set the deadzone threshold (default 0.05)
    pub fn set_deadzone(&mut self, deadzone: f32) {
        self.deadzone = deadzone.abs();
    }

    /// Set motor speed
    ///
    /// - speed: [-1.0, 1.0] where positive = forward, negative = reverse
    pub fn set(&mut self, speed: f32) {
        let speed = speed.clamp(-1.0, 1.0);

        if speed.abs() < self.deadzone {
            let _ = self.pwm.set_duty(0);
            return;
        }

        // Set direction
        if speed > 0.0 {
            self.dir.high();
        } else {
            self.dir.low();
        }

        // Set speed (0-100%)
        let duty = (speed.abs() * 100.0) as u8;
        let _ = self.pwm.set_duty(duty);
    }

    /// Stop the motor (coast)
    pub fn stop(&mut self) {
        let _ = self.pwm.set_duty(0);
    }

    /// Brake the motor (set direction and full duty, motor driver dependent)
    pub fn brake(&mut self) {
        // This is motor-driver dependent; some drivers brake when PWM is high
        // with certain DIR states. For generic use, we just stop.
        let _ = self.pwm.set_duty(0);
    }
}

/// PWM differential drive with speed control
///
/// Controls two motors (left and right) with variable speed for smooth steering.
/// Each motor uses a PWM channel for speed and a GPIO for direction.
///
/// # Example
/// ```ignore
/// let mut drive = PwmDifferentialDrive::new(left_motor, right_motor);
/// drive.arcade(0.5, 0.2);  // 50% forward, slight right turn
/// ```
pub struct PwmDifferentialDrive<'d> {
    left: PwmMotor<'d>,
    right: PwmMotor<'d>,
}

impl<'d> PwmDifferentialDrive<'d> {
    /// Create a new PWM differential drive
    pub fn new(left: PwmMotor<'d>, right: PwmMotor<'d>) -> Self {
        Self { left, right }
    }

    /// Arcade drive - throttle and steering mixed
    ///
    /// - throttle: forward/backward [-1.0, 1.0]
    /// - steering: turn left/right [-1.0, 1.0] (positive = right)
    pub fn arcade(&mut self, throttle: f32, steering: f32) {
        let throttle = throttle.clamp(-1.0, 1.0);
        let steering = steering.clamp(-1.0, 1.0);

        // Mix: steering adds to one side and subtracts from the other
        let left = (throttle + steering).clamp(-1.0, 1.0);
        let right = (throttle - steering).clamp(-1.0, 1.0);

        self.tank(left, right);
    }

    /// Tank drive - control left and right independently
    pub fn tank(&mut self, left: f32, right: f32) {
        self.left.set(left);
        self.right.set(right);
    }

    /// Stop both motors
    pub fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
    }
}

/// Car-style drive with throttle motor and steering servo
///
/// Simple car drivetrain with one motor for propulsion and one servo for steering.
///
/// # Example
/// ```ignore
/// let mut car = CarDrive::new(motor, servo);
/// car.drive(0.5, 0.2);  // 50% throttle, slight right turn
/// ```
pub struct CarDrive<'d> {
    motor: PwmMotor<'d>,
    steering: Servo<'d>,
}

impl<'d> CarDrive<'d> {
    /// Create a new car drive controller
    pub fn new(motor: PwmMotor<'d>, steering: Servo<'d>) -> Self {
        Self { motor, steering }
    }

    /// Drive the car
    ///
    /// - throttle: speed [-1.0, 1.0] (positive = forward)
    /// - steering: direction [-1.0, 1.0] (positive = right)
    pub fn drive(&mut self, throttle: f32, steering: f32) {
        self.motor.set(throttle);
        self.steering.set(steering);
    }

    /// Stop the car (motor off, steering centered)
    pub fn stop(&mut self) {
        self.motor.stop();
        self.steering.center();
    }

    /// Coast (motor off, maintain steering)
    pub fn coast(&mut self) {
        self.motor.stop();
    }

    /// Direct motor control
    pub fn motor(&mut self) -> &mut PwmMotor<'d> {
        &mut self.motor
    }

    /// Direct steering control
    pub fn steering(&mut self) -> &mut Servo<'d> {
        &mut self.steering
    }
}
