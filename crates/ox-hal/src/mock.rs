//! Mock implementations for testing without hardware
//!
//! Enable with `features = ["mock"]`

use crate::gpio::{InputPin, OutputPin, PinMode};
use crate::pwm::PwmChannel;
use crate::i2c::I2cDevice;
use crate::spi::SpiDevice;
use crate::encoder::Encoder;
use crate::motor::{Direction, Motor};

use core::cell::Cell;

/// Mock error type
#[derive(Debug, Clone, Copy)]
pub struct MockError;

/// Mock GPIO pin that tracks state in memory
pub struct MockPin {
    #[allow(dead_code)]
    is_output: bool,
    state: Cell<bool>,
}

impl MockPin {
    pub fn new(mode: PinMode) -> Self {
        Self {
            is_output: matches!(mode, PinMode::Output),
            state: Cell::new(false),
        }
    }
}

impl OutputPin for MockPin {
    type Error = MockError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state.set(true);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state.set(false);
        Ok(())
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.state.set(!self.state.get());
        Ok(())
    }

    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state.get())
    }
}

impl InputPin for MockPin {
    type Error = MockError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state.get())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state.get())
    }
}

/// Mock PWM channel
pub struct MockPwm {
    duty: u16,
    enabled: bool,
    frequency: u32,
}

impl MockPwm {
    pub fn new() -> Self {
        Self {
            duty: 0,
            enabled: false,
            frequency: 1000,
        }
    }
}

impl Default for MockPwm {
    fn default() -> Self {
        Self::new()
    }
}

impl PwmChannel for MockPwm {
    type Error = MockError;

    fn set_duty(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.duty = duty;
        Ok(())
    }

    fn get_duty(&self) -> u16 {
        self.duty
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enabled = true;
        Ok(())
    }

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.enabled = false;
        Ok(())
    }

    fn set_frequency(&mut self, freq_hz: u32) -> Result<(), Self::Error> {
        self.frequency = freq_hz;
        Ok(())
    }
}

/// Mock encoder
pub struct MockEncoder {
    position: i32,
    counts_per_rev: u32,
}

impl MockEncoder {
    pub fn new(counts_per_rev: u32) -> Self {
        Self {
            position: 0,
            counts_per_rev,
        }
    }

    /// Simulate encoder movement (for testing)
    pub fn simulate_move(&mut self, counts: i32) {
        self.position = self.position.wrapping_add(counts);
    }
}

impl Encoder for MockEncoder {
    type Error = MockError;

    fn position(&self) -> i32 {
        self.position
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        self.position = 0;
        Ok(())
    }

    fn set_position(&mut self, pos: i32) -> Result<(), Self::Error> {
        self.position = pos;
        Ok(())
    }

    fn counts_per_rev(&self) -> u32 {
        self.counts_per_rev
    }
}

/// Mock motor
pub struct MockMotor {
    speed: i16,
    direction: Direction,
}

impl MockMotor {
    pub fn new() -> Self {
        Self {
            speed: 0,
            direction: Direction::Coast,
        }
    }
}

impl Default for MockMotor {
    fn default() -> Self {
        Self::new()
    }
}

impl Motor for MockMotor {
    type Error = MockError;

    fn set_speed(&mut self, speed: i16) -> Result<(), Self::Error> {
        self.speed = speed;
        self.direction = if speed > 0 {
            Direction::Forward
        } else if speed < 0 {
            Direction::Reverse
        } else {
            Direction::Brake
        };
        Ok(())
    }

    fn speed(&self) -> i16 {
        self.speed
    }

    fn set_direction(&mut self, dir: Direction) -> Result<(), Self::Error> {
        self.direction = dir;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.speed = 0;
        self.direction = Direction::Brake;
        Ok(())
    }

    fn coast(&mut self) -> Result<(), Self::Error> {
        self.speed = 0;
        self.direction = Direction::Coast;
        Ok(())
    }
}

/// Mock I2C device with configurable responses
pub struct MockI2c {
    /// Last address written to
    pub last_addr: Cell<u8>,
    /// Buffer for simulated read responses
    read_data: [u8; 32],
    /// Number of bytes in read buffer
    read_len: usize,
    /// Last written data
    write_data: [u8; 32],
    /// Length of last write
    write_len: usize,
}

impl MockI2c {
    pub fn new() -> Self {
        Self {
            last_addr: Cell::new(0),
            read_data: [0; 32],
            read_len: 0,
            write_data: [0; 32],
            write_len: 0,
        }
    }

    /// Set data to be returned on next read
    pub fn set_read_data(&mut self, data: &[u8]) {
        let len = data.len().min(32);
        self.read_data[..len].copy_from_slice(&data[..len]);
        self.read_len = len;
    }

    /// Get last written data
    pub fn last_write(&self) -> &[u8] {
        &self.write_data[..self.write_len]
    }
}

impl Default for MockI2c {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cDevice for MockI2c {
    type Error = MockError;

    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.last_addr.set(addr);
        let len = data.len().min(32);
        self.write_data[..len].copy_from_slice(&data[..len]);
        self.write_len = len;
        Ok(())
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.last_addr.set(addr);
        let len = buffer.len().min(self.read_len);
        buffer[..len].copy_from_slice(&self.read_data[..len]);
        Ok(())
    }

    async fn write_read(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write(addr, write).await?;
        self.read(addr, read).await
    }
}

/// Mock SPI device with loopback capability
pub struct MockSpi {
    /// Last written data
    write_data: [u8; 64],
    /// Length of last write
    write_len: usize,
    /// Data to return on read (loopback by default)
    read_data: [u8; 64],
    /// Enable loopback mode (write data appears in read)
    pub loopback: bool,
}

impl MockSpi {
    pub fn new() -> Self {
        Self {
            write_data: [0; 64],
            write_len: 0,
            read_data: [0; 64],
            loopback: false,
        }
    }

    /// Set data to be returned on reads
    pub fn set_read_data(&mut self, data: &[u8]) {
        let len = data.len().min(64);
        self.read_data[..len].copy_from_slice(&data[..len]);
    }

    /// Get last written data
    pub fn last_write(&self) -> &[u8] {
        &self.write_data[..self.write_len]
    }
}

impl Default for MockSpi {
    fn default() -> Self {
        Self::new()
    }
}

impl SpiDevice for MockSpi {
    type Error = MockError;

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // Store written data
        let len = write.len().min(64);
        self.write_data[..len].copy_from_slice(&write[..len]);
        self.write_len = len;

        // Fill read buffer
        let read_len = read.len().min(64);
        if self.loopback {
            read[..read_len.min(len)].copy_from_slice(&write[..read_len.min(len)]);
        } else {
            read[..read_len].copy_from_slice(&self.read_data[..read_len]);
        }
        Ok(())
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        let len = data.len().min(64);
        self.write_data[..len].copy_from_slice(&data[..len]);
        self.write_len = len;
        Ok(())
    }

    async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let len = buffer.len().min(64);
        buffer[..len].copy_from_slice(&self.read_data[..len]);
        Ok(())
    }

    async fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        let len = data.len().min(64);
        self.write_data[..len].copy_from_slice(&data[..len]);
        self.write_len = len;

        if !self.loopback {
            data[..len].copy_from_slice(&self.read_data[..len]);
        }
        // In loopback mode, data stays the same
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== MockPin Tests ====================

    #[test]
    fn mock_pin_starts_low() {
        let pin = MockPin::new(PinMode::Output);
        assert!(!pin.state.get());
        assert!(pin.is_set_high().unwrap() == false);
    }

    #[test]
    fn mock_pin_set_high() {
        let mut pin = MockPin::new(PinMode::Output);
        pin.set_high().unwrap();
        assert!(pin.state.get());
        assert!(pin.is_set_high().unwrap());
    }

    #[test]
    fn mock_pin_set_low() {
        let mut pin = MockPin::new(PinMode::Output);
        pin.set_high().unwrap();
        pin.set_low().unwrap();
        assert!(!pin.state.get());
    }

    #[test]
    fn mock_pin_toggles() {
        let mut pin = MockPin::new(PinMode::Output);
        assert!(!pin.state.get());
        pin.set_high().unwrap();
        assert!(pin.state.get());
        pin.toggle().unwrap();
        assert!(!pin.state.get());
        pin.toggle().unwrap();
        assert!(pin.state.get());
    }

    #[test]
    fn mock_pin_input_reads_state() {
        let pin = MockPin::new(PinMode::Input);
        assert!(pin.is_low().unwrap());
        assert!(!pin.is_high().unwrap());

        // Simulate external signal by setting state directly
        pin.state.set(true);
        assert!(pin.is_high().unwrap());
        assert!(!pin.is_low().unwrap());
    }

    #[test]
    fn mock_pin_modes() {
        let _input = MockPin::new(PinMode::Input);
        let _output = MockPin::new(PinMode::Output);
        let _pullup = MockPin::new(PinMode::InputPullUp);
        let _pulldown = MockPin::new(PinMode::InputPullDown);
    }

    // ==================== MockPwm Tests ====================

    #[test]
    fn mock_pwm_default_duty_zero() {
        let pwm = MockPwm::new();
        assert_eq!(pwm.get_duty(), 0);
    }

    #[test]
    fn mock_pwm_sets_duty() {
        let mut pwm = MockPwm::new();
        pwm.set_duty(32768).unwrap();
        assert_eq!(pwm.get_duty(), 32768);
    }

    #[test]
    fn mock_pwm_full_duty() {
        let mut pwm = MockPwm::new();
        pwm.set_duty(65535).unwrap();
        assert_eq!(pwm.get_duty(), 65535);
    }

    #[test]
    fn mock_pwm_enable_disable() {
        let mut pwm = MockPwm::new();
        assert!(!pwm.enabled);
        pwm.enable().unwrap();
        assert!(pwm.enabled);
        pwm.disable().unwrap();
        assert!(!pwm.enabled);
    }

    #[test]
    fn mock_pwm_set_frequency() {
        let mut pwm = MockPwm::new();
        assert_eq!(pwm.frequency, 1000); // Default
        pwm.set_frequency(20000).unwrap();
        assert_eq!(pwm.frequency, 20000);
    }

    #[test]
    fn mock_pwm_default_trait() {
        let pwm = MockPwm::default();
        assert_eq!(pwm.get_duty(), 0);
        assert!(!pwm.enabled);
    }

    // ==================== MockEncoder Tests ====================

    #[test]
    fn mock_encoder_starts_at_zero() {
        let enc = MockEncoder::new(1000);
        assert_eq!(enc.position(), 0);
    }

    #[test]
    fn mock_encoder_tracks_position() {
        let mut enc = MockEncoder::new(1000);
        enc.simulate_move(100);
        assert_eq!(enc.position(), 100);
        enc.simulate_move(-50);
        assert_eq!(enc.position(), 50);
    }

    #[test]
    fn mock_encoder_negative_position() {
        let mut enc = MockEncoder::new(1000);
        enc.simulate_move(-100);
        assert_eq!(enc.position(), -100);
    }

    #[test]
    fn mock_encoder_reset() {
        let mut enc = MockEncoder::new(1000);
        enc.simulate_move(500);
        assert_eq!(enc.position(), 500);
        enc.reset().unwrap();
        assert_eq!(enc.position(), 0);
    }

    #[test]
    fn mock_encoder_set_position() {
        let mut enc = MockEncoder::new(1000);
        enc.set_position(12345).unwrap();
        assert_eq!(enc.position(), 12345);
    }

    #[test]
    fn mock_encoder_counts_per_rev() {
        let enc1 = MockEncoder::new(1000);
        let enc2 = MockEncoder::new(4096);
        assert_eq!(enc1.counts_per_rev(), 1000);
        assert_eq!(enc2.counts_per_rev(), 4096);
    }

    #[test]
    fn mock_encoder_wrapping() {
        let mut enc = MockEncoder::new(1000);
        enc.set_position(i32::MAX).unwrap();
        enc.simulate_move(1);
        assert_eq!(enc.position(), i32::MIN);
    }

    // ==================== MockMotor Tests ====================

    #[test]
    fn mock_motor_starts_stopped() {
        let motor = MockMotor::new();
        assert_eq!(motor.speed(), 0);
        assert_eq!(motor.direction, Direction::Coast);
    }

    #[test]
    fn mock_motor_sets_speed() {
        let mut motor = MockMotor::new();
        motor.set_speed(1000).unwrap();
        assert_eq!(motor.speed(), 1000);
    }

    #[test]
    fn mock_motor_forward_direction() {
        let mut motor = MockMotor::new();
        motor.set_speed(100).unwrap();
        assert_eq!(motor.direction, Direction::Forward);
    }

    #[test]
    fn mock_motor_reverse_direction() {
        let mut motor = MockMotor::new();
        motor.set_speed(-100).unwrap();
        assert_eq!(motor.direction, Direction::Reverse);
    }

    #[test]
    fn mock_motor_zero_speed_brakes() {
        let mut motor = MockMotor::new();
        motor.set_speed(100).unwrap();
        motor.set_speed(0).unwrap();
        assert_eq!(motor.direction, Direction::Brake);
    }

    #[test]
    fn mock_motor_stop() {
        let mut motor = MockMotor::new();
        motor.set_speed(1000).unwrap();
        motor.stop().unwrap();
        assert_eq!(motor.speed(), 0);
        assert_eq!(motor.direction, Direction::Brake);
    }

    #[test]
    fn mock_motor_coast() {
        let mut motor = MockMotor::new();
        motor.set_speed(1000).unwrap();
        motor.coast().unwrap();
        assert_eq!(motor.speed(), 0);
        assert_eq!(motor.direction, Direction::Coast);
    }

    #[test]
    fn mock_motor_set_direction() {
        let mut motor = MockMotor::new();
        motor.set_direction(Direction::Forward).unwrap();
        assert_eq!(motor.direction, Direction::Forward);
        motor.set_direction(Direction::Reverse).unwrap();
        assert_eq!(motor.direction, Direction::Reverse);
    }

    #[test]
    fn mock_motor_full_speed() {
        let mut motor = MockMotor::new();
        motor.set_speed(i16::MAX).unwrap();
        assert_eq!(motor.speed(), i16::MAX);
        motor.set_speed(i16::MIN).unwrap();
        assert_eq!(motor.speed(), i16::MIN);
    }

    #[test]
    fn mock_motor_default_trait() {
        let motor = MockMotor::default();
        assert_eq!(motor.speed(), 0);
    }

    // ==================== MockI2c Tests ====================

    #[tokio::test]
    async fn mock_i2c_write() {
        let mut i2c = MockI2c::new();
        i2c.write(0x50, &[0x00, 0x01, 0x02]).await.unwrap();
        assert_eq!(i2c.last_addr.get(), 0x50);
        assert_eq!(i2c.last_write(), &[0x00, 0x01, 0x02]);
    }

    #[tokio::test]
    async fn mock_i2c_read() {
        let mut i2c = MockI2c::new();
        i2c.set_read_data(&[0xAA, 0xBB, 0xCC]);

        let mut buf = [0u8; 3];
        i2c.read(0x68, &mut buf).await.unwrap();

        assert_eq!(i2c.last_addr.get(), 0x68);
        assert_eq!(buf, [0xAA, 0xBB, 0xCC]);
    }

    #[tokio::test]
    async fn mock_i2c_write_read() {
        let mut i2c = MockI2c::new();
        i2c.set_read_data(&[0x12, 0x34]);

        let mut buf = [0u8; 2];
        i2c.write_read(0x50, &[0x00], &mut buf).await.unwrap();

        assert_eq!(i2c.last_write(), &[0x00]);
        assert_eq!(buf, [0x12, 0x34]);
    }

    #[tokio::test]
    async fn mock_i2c_default() {
        let i2c = MockI2c::default();
        assert_eq!(i2c.last_addr.get(), 0);
    }

    // ==================== MockSpi Tests ====================

    #[tokio::test]
    async fn mock_spi_write() {
        let mut spi = MockSpi::new();
        spi.write(&[0x01, 0x02, 0x03]).await.unwrap();
        assert_eq!(spi.last_write(), &[0x01, 0x02, 0x03]);
    }

    #[tokio::test]
    async fn mock_spi_read() {
        let mut spi = MockSpi::new();
        spi.set_read_data(&[0xDE, 0xAD, 0xBE, 0xEF]);

        let mut buf = [0u8; 4];
        spi.read(&mut buf).await.unwrap();
        assert_eq!(buf, [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[tokio::test]
    async fn mock_spi_transfer() {
        let mut spi = MockSpi::new();
        spi.set_read_data(&[0xAA, 0xBB]);

        let mut read = [0u8; 2];
        spi.transfer(&mut read, &[0x01, 0x02]).await.unwrap();

        assert_eq!(spi.last_write(), &[0x01, 0x02]);
        assert_eq!(read, [0xAA, 0xBB]);
    }

    #[tokio::test]
    async fn mock_spi_loopback() {
        let mut spi = MockSpi::new();
        spi.loopback = true;

        let mut read = [0u8; 3];
        spi.transfer(&mut read, &[0x11, 0x22, 0x33]).await.unwrap();
        assert_eq!(read, [0x11, 0x22, 0x33]);
    }

    #[tokio::test]
    async fn mock_spi_transfer_in_place() {
        let mut spi = MockSpi::new();
        spi.set_read_data(&[0xFF, 0xEE]);

        let mut data = [0x01, 0x02];
        spi.transfer_in_place(&mut data).await.unwrap();

        assert_eq!(spi.last_write(), &[0x01, 0x02]);
        assert_eq!(data, [0xFF, 0xEE]);
    }

    #[tokio::test]
    async fn mock_spi_transfer_in_place_loopback() {
        let mut spi = MockSpi::new();
        spi.loopback = true;

        let mut data = [0x42, 0x43];
        spi.transfer_in_place(&mut data).await.unwrap();

        // Data stays the same in loopback mode
        assert_eq!(data, [0x42, 0x43]);
    }

    #[test]
    fn mock_spi_default() {
        let spi = MockSpi::default();
        assert!(!spi.loopback);
        assert_eq!(spi.write_len, 0);
    }

    // ==================== Direction Enum Tests ====================

    #[test]
    fn direction_variants() {
        assert_eq!(Direction::Forward, Direction::Forward);
        assert_eq!(Direction::Reverse, Direction::Reverse);
        assert_eq!(Direction::Brake, Direction::Brake);
        assert_eq!(Direction::Coast, Direction::Coast);
    }

    #[test]
    fn direction_inequality() {
        assert_ne!(Direction::Forward, Direction::Reverse);
        assert_ne!(Direction::Brake, Direction::Coast);
    }

    #[test]
    fn direction_copy() {
        let d1 = Direction::Forward;
        let d2 = d1;
        assert_eq!(d1, d2);
    }

    // ==================== PinMode Enum Tests ====================

    #[test]
    fn pin_mode_variants() {
        assert_eq!(PinMode::Input, PinMode::Input);
        assert_eq!(PinMode::Output, PinMode::Output);
        assert_eq!(PinMode::InputPullUp, PinMode::InputPullUp);
        assert_eq!(PinMode::InputPullDown, PinMode::InputPullDown);
    }

    #[test]
    fn pin_mode_copy() {
        let m1 = PinMode::Output;
        let m2 = m1;
        assert_eq!(m1, m2);
    }

    // ==================== MockError Tests ====================

    #[test]
    fn mock_error_debug() {
        extern crate std;
        use std::format;
        let err = MockError;
        let debug = format!("{:?}", err);
        assert!(debug.contains("MockError"));
    }

    #[test]
    fn mock_error_copy() {
        let e1 = MockError;
        let e2 = e1;
        let _ = (e1, e2);
    }
}
