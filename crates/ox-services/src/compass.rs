//! Compass (Magnetometer) Server
//!
//! Provides heading data from magnetometer sensors.
//!
//! Supported sensors:
//! - QMC5883L (common cheap compass, I2C address 0x0D)
//! - HMC5883L (Honeywell, I2C address 0x1E)
//!
//! Features:
//! - Magnetic heading (0-360 degrees)
//! - True heading (corrected for declination)
//! - Hard-iron calibration (runtime min/max tracking)

use core::f32::consts::PI;

/// Raw magnetometer data
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MagRaw {
    /// X-axis raw value
    pub x: i16,
    /// Y-axis raw value
    pub y: i16,
    /// Z-axis raw value
    pub z: i16,
}

/// Calibration status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CalibrationStatus {
    /// No calibration data collected
    #[default]
    Uncalibrated,
    /// Currently collecting calibration samples
    Collecting,
    /// Calibration complete
    Calibrated,
}

/// Hard-iron calibration offsets
#[derive(Debug, Clone, Copy, Default)]
pub struct HardIronCal {
    /// X-axis offset (center of min/max range)
    pub x_offset: i16,
    /// Y-axis offset
    pub y_offset: i16,
    /// Z-axis offset
    pub z_offset: i16,
}

/// Compass output data
#[derive(Debug, Clone, Copy, Default)]
pub struct CompassData {
    /// Magnetic heading in degrees (0-360, 0 = magnetic north)
    pub heading: f32,
    /// True heading in degrees (corrected for declination)
    pub heading_true: f32,
    /// Raw magnetometer reading
    pub raw: MagRaw,
    /// Calibration status
    pub calibration: CalibrationStatus,
    /// Data validity flag
    pub valid: bool,
}

/// Compass configuration
#[derive(Debug, Clone, Copy)]
pub struct CompassConfig {
    /// Local magnetic declination in degrees
    /// Positive = east declination, negative = west
    /// Look up at: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    pub declination: f32,
    /// Minimum samples required for calibration
    pub min_cal_samples: u16,
}

impl Default for CompassConfig {
    fn default() -> Self {
        Self {
            declination: 0.0,
            min_cal_samples: 100,
        }
    }
}

// ============================================================================
// Compass Server
// ============================================================================

/// Compass server that processes magnetometer data
///
/// Handles calibration and heading calculation.
pub struct CompassServer {
    config: CompassConfig,
    calibration: HardIronCal,
    cal_status: CalibrationStatus,
    // Calibration tracking (min/max per axis)
    min_raw: MagRaw,
    max_raw: MagRaw,
    samples_collected: u16,
    last_data: CompassData,
}

impl CompassServer {
    /// Create a new compass server
    pub fn new(config: CompassConfig) -> Self {
        Self {
            config,
            calibration: HardIronCal::default(),
            cal_status: CalibrationStatus::Uncalibrated,
            min_raw: MagRaw { x: i16::MAX, y: i16::MAX, z: i16::MAX },
            max_raw: MagRaw { x: i16::MIN, y: i16::MIN, z: i16::MIN },
            samples_collected: 0,
            last_data: CompassData::default(),
        }
    }

    /// Process raw magnetometer data
    ///
    /// Returns compass data with heading calculated from the raw values.
    pub fn process(&mut self, raw: MagRaw) -> CompassData {
        // Update calibration tracking if collecting
        if self.cal_status == CalibrationStatus::Collecting {
            self.update_cal_tracking(&raw);
        }

        // Calculate heading
        let heading = self.compute_heading(&raw);
        let heading_true = self.apply_declination(heading);

        self.last_data = CompassData {
            heading,
            heading_true,
            raw,
            calibration: self.cal_status,
            valid: true,
        };

        self.last_data
    }

    /// Start calibration routine
    ///
    /// User should slowly rotate the device through all orientations
    /// while calibration is collecting.
    pub fn start_calibration(&mut self) {
        self.cal_status = CalibrationStatus::Collecting;
        self.min_raw = MagRaw { x: i16::MAX, y: i16::MAX, z: i16::MAX };
        self.max_raw = MagRaw { x: i16::MIN, y: i16::MIN, z: i16::MIN };
        self.samples_collected = 0;
    }

    /// Finish calibration and compute offsets
    ///
    /// Returns the computed hard-iron calibration values.
    pub fn finish_calibration(&mut self) -> HardIronCal {
        if self.samples_collected >= self.config.min_cal_samples {
            // Compute center of min/max range as offset
            self.calibration = HardIronCal {
                x_offset: ((self.min_raw.x as i32 + self.max_raw.x as i32) / 2) as i16,
                y_offset: ((self.min_raw.y as i32 + self.max_raw.y as i32) / 2) as i16,
                z_offset: ((self.min_raw.z as i32 + self.max_raw.z as i32) / 2) as i16,
            };
            self.cal_status = CalibrationStatus::Calibrated;
        }
        self.calibration
    }

    /// Set calibration values directly
    ///
    /// Use this to restore previously computed calibration values.
    pub fn set_calibration(&mut self, cal: HardIronCal) {
        self.calibration = cal;
        self.cal_status = CalibrationStatus::Calibrated;
    }

    /// Get current calibration values
    pub fn calibration(&self) -> &HardIronCal {
        &self.calibration
    }

    /// Get calibration status
    pub fn calibration_status(&self) -> CalibrationStatus {
        self.cal_status
    }

    /// Get number of calibration samples collected
    pub fn calibration_samples(&self) -> u16 {
        self.samples_collected
    }

    /// Get last computed data
    pub fn last_data(&self) -> &CompassData {
        &self.last_data
    }

    /// Update min/max tracking for calibration
    fn update_cal_tracking(&mut self, raw: &MagRaw) {
        self.min_raw.x = self.min_raw.x.min(raw.x);
        self.min_raw.y = self.min_raw.y.min(raw.y);
        self.min_raw.z = self.min_raw.z.min(raw.z);

        self.max_raw.x = self.max_raw.x.max(raw.x);
        self.max_raw.y = self.max_raw.y.max(raw.y);
        self.max_raw.z = self.max_raw.z.max(raw.z);

        self.samples_collected = self.samples_collected.saturating_add(1);
    }

    /// Compute heading from raw magnetometer data
    ///
    /// Returns heading in degrees (0-360), where 0 = magnetic north.
    fn compute_heading(&self, raw: &MagRaw) -> f32 {
        // Apply hard-iron correction
        let x = (raw.x - self.calibration.x_offset) as f32;
        let y = (raw.y - self.calibration.y_offset) as f32;

        // atan2 gives radians from -PI to +PI
        // For a flat compass: heading = atan2(east, north) = atan2(y, x)
        // But sensor orientation matters - this assumes:
        // - X points forward (north when heading=0)
        // - Y points right (east)
        let heading_rad = libm::atan2f(y, x);

        // Convert to degrees (0-360)
        let mut heading_deg = heading_rad * 180.0 / PI;
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        heading_deg
    }

    /// Apply magnetic declination to get true heading
    fn apply_declination(&self, magnetic_heading: f32) -> f32 {
        let mut true_heading = magnetic_heading + self.config.declination;

        // Normalize to 0-360
        if true_heading < 0.0 {
            true_heading += 360.0;
        } else if true_heading >= 360.0 {
            true_heading -= 360.0;
        }

        true_heading
    }
}

impl Default for CompassServer {
    fn default() -> Self {
        Self::new(CompassConfig::default())
    }
}

// ============================================================================
// QMC5883L Driver
// ============================================================================

/// QMC5883L I2C address
pub const QMC5883L_ADDR: u8 = 0x0D;

/// QMC5883L registers
mod qmc5883l_regs {
    pub const DATA_X_LSB: u8 = 0x00;
    pub const STATUS: u8 = 0x06;
    pub const CONTROL1: u8 = 0x09;
    pub const CONTROL2: u8 = 0x0A;
    pub const SET_RESET: u8 = 0x0B;
}

/// QMC5883L output data rate
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Qmc5883lOdr {
    /// 10 Hz
    Hz10 = 0x00,
    /// 50 Hz
    Hz50 = 0x04,
    /// 100 Hz
    #[default]
    Hz100 = 0x08,
    /// 200 Hz
    Hz200 = 0x0C,
}

/// QMC5883L full scale range
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Qmc5883lRange {
    /// ±2 Gauss
    #[default]
    Gauss2 = 0x00,
    /// ±8 Gauss
    Gauss8 = 0x10,
}

/// QMC5883L oversampling ratio
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Qmc5883lOsr {
    /// 512 samples
    #[default]
    Osr512 = 0x00,
    /// 256 samples
    Osr256 = 0x40,
    /// 128 samples
    Osr128 = 0x80,
    /// 64 samples
    Osr64 = 0xC0,
}

/// QMC5883L driver error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Qmc5883lError<E> {
    /// I2C communication error
    I2c(E),
    /// Data not ready
    NotReady,
}

/// QMC5883L magnetometer driver
pub struct Qmc5883l<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Qmc5883l<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// Create a new QMC5883L driver
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: QMC5883L_ADDR,
        }
    }

    /// Create with custom address (rare)
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Initialize the sensor with default settings
    ///
    /// Default: continuous mode, 100Hz, ±2G, OSR512
    pub fn init(&mut self) -> Result<(), Qmc5883lError<E>> {
        self.init_with_config(Qmc5883lOdr::Hz100, Qmc5883lRange::Gauss2, Qmc5883lOsr::Osr512)
    }

    /// Initialize with specific configuration
    pub fn init_with_config(
        &mut self,
        odr: Qmc5883lOdr,
        range: Qmc5883lRange,
        osr: Qmc5883lOsr,
    ) -> Result<(), Qmc5883lError<E>> {
        // Soft reset
        self.write_register(qmc5883l_regs::CONTROL2, 0x80)?;

        // Set/Reset period
        self.write_register(qmc5883l_regs::SET_RESET, 0x01)?;

        // Configure: continuous mode + ODR + range + OSR
        let ctrl1 = 0x01 | (odr as u8) | (range as u8) | (osr as u8);
        self.write_register(qmc5883l_regs::CONTROL1, ctrl1)?;

        Ok(())
    }

    /// Read magnetometer data
    ///
    /// Returns raw X, Y, Z values in sensor units.
    pub fn read(&mut self) -> Result<MagRaw, Qmc5883lError<E>> {
        // Check status
        let status = self.read_register(qmc5883l_regs::STATUS)?;
        if (status & 0x01) == 0 {
            return Err(Qmc5883lError::NotReady);
        }

        // Read 6 bytes starting from DATA_X_LSB
        let mut buf = [0u8; 6];
        self.read_registers(qmc5883l_regs::DATA_X_LSB, &mut buf)?;

        // QMC5883L data is little-endian
        Ok(MagRaw {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Qmc5883lError<E>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Qmc5883lError::I2c)
    }

    fn read_register(&mut self, reg: u8) -> Result<u8, Qmc5883lError<E>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Qmc5883lError::I2c)?;
        Ok(buf[0])
    }

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Qmc5883lError<E>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(Qmc5883lError::I2c)
    }
}

// ============================================================================
// HMC5883L Driver (optional, for compatibility)
// ============================================================================

/// HMC5883L I2C address
pub const HMC5883L_ADDR: u8 = 0x1E;

/// HMC5883L registers
mod hmc5883l_regs {
    pub const CONFIG_A: u8 = 0x00;
    pub const CONFIG_B: u8 = 0x01;
    pub const MODE: u8 = 0x02;
    pub const DATA_X_MSB: u8 = 0x03;
    pub const STATUS: u8 = 0x09;
}

/// HMC5883L driver error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hmc5883lError<E> {
    /// I2C communication error
    I2c(E),
    /// Data not ready
    NotReady,
}

/// HMC5883L magnetometer driver
pub struct Hmc5883l<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Hmc5883l<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    /// Create a new HMC5883L driver
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: HMC5883L_ADDR,
        }
    }

    /// Initialize the sensor
    ///
    /// Default: 8 samples average, 15Hz, ±1.3Ga, continuous mode
    pub fn init(&mut self) -> Result<(), Hmc5883lError<E>> {
        // Config A: 8 samples avg, 15Hz output, normal measurement
        self.write_register(hmc5883l_regs::CONFIG_A, 0x70)?;

        // Config B: Gain = 1.3Ga (default)
        self.write_register(hmc5883l_regs::CONFIG_B, 0x20)?;

        // Mode: Continuous measurement
        self.write_register(hmc5883l_regs::MODE, 0x00)?;

        Ok(())
    }

    /// Read magnetometer data
    pub fn read(&mut self) -> Result<MagRaw, Hmc5883lError<E>> {
        // Check status
        let status = self.read_register(hmc5883l_regs::STATUS)?;
        if (status & 0x01) == 0 {
            return Err(Hmc5883lError::NotReady);
        }

        // Read 6 bytes starting from DATA_X_MSB
        // Note: HMC5883L has X, Z, Y order (not X, Y, Z!)
        let mut buf = [0u8; 6];
        self.read_registers(hmc5883l_regs::DATA_X_MSB, &mut buf)?;

        // HMC5883L data is big-endian, and order is X, Z, Y
        Ok(MagRaw {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            z: i16::from_be_bytes([buf[2], buf[3]]),
            y: i16::from_be_bytes([buf[4], buf[5]]),
        })
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Hmc5883lError<E>> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(Hmc5883lError::I2c)
    }

    fn read_register(&mut self, reg: u8) -> Result<u8, Hmc5883lError<E>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(Hmc5883lError::I2c)?;
        Ok(buf[0])
    }

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Hmc5883lError<E>> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(Hmc5883lError::I2c)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compass_heading_north() {
        let mut server = CompassServer::new(CompassConfig::default());

        // Pointing magnetic north: X positive, Y zero
        let raw = MagRaw { x: 1000, y: 0, z: 0 };
        let data = server.process(raw);

        // Should be close to 0 degrees
        assert!(data.heading < 1.0 || data.heading > 359.0);
        assert!(data.valid);
    }

    #[test]
    fn test_compass_heading_east() {
        let mut server = CompassServer::new(CompassConfig::default());

        // Pointing east: X zero, Y positive
        let raw = MagRaw { x: 0, y: 1000, z: 0 };
        let data = server.process(raw);

        assert!((data.heading - 90.0).abs() < 1.0);
    }

    #[test]
    fn test_compass_heading_south() {
        let mut server = CompassServer::new(CompassConfig::default());

        // Pointing south: X negative, Y zero
        let raw = MagRaw { x: -1000, y: 0, z: 0 };
        let data = server.process(raw);

        assert!((data.heading - 180.0).abs() < 1.0);
    }

    #[test]
    fn test_compass_heading_west() {
        let mut server = CompassServer::new(CompassConfig::default());

        // Pointing west: X zero, Y negative
        let raw = MagRaw { x: 0, y: -1000, z: 0 };
        let data = server.process(raw);

        assert!((data.heading - 270.0).abs() < 1.0);
    }

    #[test]
    fn test_compass_declination() {
        let config = CompassConfig {
            declination: 10.0, // 10 degrees east
            ..Default::default()
        };
        let mut server = CompassServer::new(config);

        // Pointing magnetic north
        let raw = MagRaw { x: 1000, y: 0, z: 0 };
        let data = server.process(raw);

        // Magnetic heading ~0, true heading ~10
        assert!(data.heading < 1.0 || data.heading > 359.0);
        assert!((data.heading_true - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_compass_declination_negative() {
        let config = CompassConfig {
            declination: -15.0, // 15 degrees west
            ..Default::default()
        };
        let mut server = CompassServer::new(config);

        // Pointing magnetic north
        let raw = MagRaw { x: 1000, y: 0, z: 0 };
        let data = server.process(raw);

        // Magnetic heading ~0, true heading ~345
        assert!((data.heading_true - 345.0).abs() < 1.0);
    }

    #[test]
    fn test_calibration_process() {
        let config = CompassConfig {
            min_cal_samples: 4,
            ..Default::default()
        };
        let mut server = CompassServer::new(config);

        server.start_calibration();
        assert_eq!(server.calibration_status(), CalibrationStatus::Collecting);

        // Simulate rotation with offset center at (100, 200)
        let readings = [
            MagRaw { x: 0, y: 100, z: 0 },     // Min X
            MagRaw { x: 200, y: 100, z: 0 },   // Max X
            MagRaw { x: 100, y: 0, z: 0 },     // Min Y
            MagRaw { x: 100, y: 400, z: 0 },   // Max Y
        ];

        for raw in readings {
            server.process(raw);
        }

        let cal = server.finish_calibration();
        assert_eq!(server.calibration_status(), CalibrationStatus::Calibrated);

        // Center should be (100, 200)
        assert_eq!(cal.x_offset, 100);
        assert_eq!(cal.y_offset, 200);
    }

    #[test]
    fn test_calibration_applied() {
        let mut server = CompassServer::new(CompassConfig::default());

        // Set calibration with offset
        server.set_calibration(HardIronCal {
            x_offset: 100,
            y_offset: 0,
            z_offset: 0,
        });

        // Raw reading that would be off if uncalibrated
        // After correction: x = 1100 - 100 = 1000, y = 0
        let raw = MagRaw { x: 1100, y: 0, z: 0 };
        let data = server.process(raw);

        // Should point north after calibration
        assert!(data.heading < 1.0 || data.heading > 359.0);
    }

    #[test]
    fn test_calibration_not_enough_samples() {
        let config = CompassConfig {
            min_cal_samples: 100,
            ..Default::default()
        };
        let mut server = CompassServer::new(config);

        server.start_calibration();

        // Only 2 samples
        server.process(MagRaw { x: 0, y: 0, z: 0 });
        server.process(MagRaw { x: 100, y: 100, z: 100 });

        let cal = server.finish_calibration();

        // Should not update calibration (still Collecting status becomes Calibrated
        // only if enough samples)
        // Actually looking at the code, it stays Collecting if not enough samples
        assert_eq!(server.calibration_status(), CalibrationStatus::Collecting);
        assert_eq!(cal.x_offset, 0); // Not updated
    }

    #[test]
    fn test_mag_raw_default() {
        let raw = MagRaw::default();
        assert_eq!(raw.x, 0);
        assert_eq!(raw.y, 0);
        assert_eq!(raw.z, 0);
    }
}
