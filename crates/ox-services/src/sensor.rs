//! Sensor fusion server
//!
//! Handles IMU, encoders, and other sensors.
//!
//! Features:
//! - IMU data processing (accelerometer, gyroscope)
//! - Attitude estimation via complementary filter
//! - Encoder position and velocity tracking
//! - Odometry calculation

use ox_control::filter::ComplementaryFilter;

use core::f32::consts::PI;

/// Gravity constant (m/s²)
pub const GRAVITY: f32 = 9.81;

/// IMU raw data (from sensor)
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuRaw {
    /// Accelerometer X (raw counts)
    pub accel_x: i16,
    /// Accelerometer Y (raw counts)
    pub accel_y: i16,
    /// Accelerometer Z (raw counts)
    pub accel_z: i16,
    /// Gyroscope X (raw counts)
    pub gyro_x: i16,
    /// Gyroscope Y (raw counts)
    pub gyro_y: i16,
    /// Gyroscope Z (raw counts)
    pub gyro_z: i16,
}

/// IMU calibrated data (engineering units)
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Accelerometer X (m/s²)
    pub accel_x: f32,
    /// Accelerometer Y (m/s²)
    pub accel_y: f32,
    /// Accelerometer Z (m/s²)
    pub accel_z: f32,
    /// Gyroscope X (rad/s)
    pub gyro_x: f32,
    /// Gyroscope Y (rad/s)
    pub gyro_y: f32,
    /// Gyroscope Z (rad/s)
    pub gyro_z: f32,
}

/// Fused attitude estimate
#[derive(Debug, Clone, Copy, Default)]
pub struct Attitude {
    /// Roll angle (radians) - rotation about X axis
    pub roll: f32,
    /// Pitch angle (radians) - rotation about Y axis
    pub pitch: f32,
    /// Yaw angle (radians) - rotation about Z axis
    pub yaw: f32,
}

/// Encoder readings
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EncoderData {
    pub left_position: i32,
    pub right_position: i32,
    pub left_velocity: i16,
    pub right_velocity: i16,
}

/// 2D pose (position and heading)
#[derive(Debug, Clone, Copy, Default)]
pub struct Pose2D {
    /// X position (meters)
    pub x: f32,
    /// Y position (meters)
    pub y: f32,
    /// Heading (radians)
    pub theta: f32,
}

/// IMU calibration parameters
#[derive(Debug, Clone, Copy)]
pub struct ImuCalibration {
    /// Accelerometer scale (m/s² per count)
    pub accel_scale: f32,
    /// Gyroscope scale (rad/s per count)
    pub gyro_scale: f32,
    /// Accelerometer offsets
    pub accel_offset: (i16, i16, i16),
    /// Gyroscope offsets
    pub gyro_offset: (i16, i16, i16),
}

impl Default for ImuCalibration {
    fn default() -> Self {
        // MPU6050 defaults: ±2g, ±250°/s
        Self {
            accel_scale: GRAVITY / 16384.0,  // 2g / 32768
            gyro_scale: (250.0 * PI / 180.0) / 32768.0,  // 250°/s in rad/s
            accel_offset: (0, 0, 0),
            gyro_offset: (0, 0, 0),
        }
    }
}

/// Odometry configuration
#[derive(Debug, Clone, Copy)]
pub struct OdometryConfig {
    /// Wheel radius (meters)
    pub wheel_radius: f32,
    /// Track width (meters) - distance between wheels
    pub track_width: f32,
    /// Encoder counts per wheel revolution
    pub counts_per_rev: u32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            wheel_radius: 0.05,     // 5cm radius
            track_width: 0.20,      // 20cm track
            counts_per_rev: 1000,   // 1000 CPR encoder
        }
    }
}

/// Sensor server core logic
pub struct SensorServer {
    // IMU processing
    imu_cal: ImuCalibration,
    roll_filter: ComplementaryFilter,
    pitch_filter: ComplementaryFilter,
    attitude: Attitude,

    // Encoder tracking
    odom_config: OdometryConfig,
    last_encoder: EncoderData,
    pose: Pose2D,
}

impl SensorServer {
    /// Create a new sensor server
    pub fn new(imu_cal: ImuCalibration, odom_config: OdometryConfig) -> Self {
        Self {
            imu_cal,
            roll_filter: ComplementaryFilter::new(0.98),
            pitch_filter: ComplementaryFilter::new(0.98),
            attitude: Attitude::default(),
            odom_config,
            last_encoder: EncoderData::default(),
            pose: Pose2D::default(),
        }
    }

    /// Process raw IMU data and update attitude
    pub fn process_imu(&mut self, raw: ImuRaw, dt: f32) -> ImuData {
        // Apply calibration
        let imu = self.calibrate_imu(raw);

        // Calculate accelerometer-based angles
        let accel_roll = libm::atan2f(imu.accel_y, imu.accel_z);
        let accel_pitch = libm::atan2f(
            -imu.accel_x,
            libm::sqrtf(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z),
        );

        // Convert gyro rates to degrees/s for filter
        let gyro_roll_dps = imu.gyro_x * 180.0 / PI;
        let gyro_pitch_dps = imu.gyro_y * 180.0 / PI;

        // Update complementary filters
        let roll_deg = self.roll_filter.update(accel_roll * 180.0 / PI, gyro_roll_dps, dt);
        let pitch_deg = self.pitch_filter.update(accel_pitch * 180.0 / PI, gyro_pitch_dps, dt);

        // Store attitude in radians
        self.attitude.roll = roll_deg * PI / 180.0;
        self.attitude.pitch = pitch_deg * PI / 180.0;
        // Yaw from gyro integration only (no magnetometer)
        self.attitude.yaw += imu.gyro_z * dt;

        imu
    }

    /// Calibrate raw IMU data to engineering units
    pub fn calibrate_imu(&self, raw: ImuRaw) -> ImuData {
        let cal = &self.imu_cal;

        ImuData {
            accel_x: (raw.accel_x - cal.accel_offset.0) as f32 * cal.accel_scale,
            accel_y: (raw.accel_y - cal.accel_offset.1) as f32 * cal.accel_scale,
            accel_z: (raw.accel_z - cal.accel_offset.2) as f32 * cal.accel_scale,
            gyro_x: (raw.gyro_x - cal.gyro_offset.0) as f32 * cal.gyro_scale,
            gyro_y: (raw.gyro_y - cal.gyro_offset.1) as f32 * cal.gyro_scale,
            gyro_z: (raw.gyro_z - cal.gyro_offset.2) as f32 * cal.gyro_scale,
        }
    }

    /// Process encoder data and update odometry
    pub fn process_encoders(&mut self, current: EncoderData, _dt: f32) -> Pose2D {
        let cfg = &self.odom_config;

        // Calculate wheel deltas (in counts)
        let left_delta = current.left_position - self.last_encoder.left_position;
        let right_delta = current.right_position - self.last_encoder.right_position;
        self.last_encoder = current;

        // Convert to distance (meters)
        let meters_per_count = 2.0 * PI * cfg.wheel_radius / cfg.counts_per_rev as f32;
        let left_dist = left_delta as f32 * meters_per_count;
        let right_dist = right_delta as f32 * meters_per_count;

        // Differential drive kinematics
        let linear = (left_dist + right_dist) / 2.0;
        let angular = (right_dist - left_dist) / cfg.track_width;

        // Update pose
        self.pose.x += linear * libm::cosf(self.pose.theta);
        self.pose.y += linear * libm::sinf(self.pose.theta);
        self.pose.theta += angular;

        // Normalize theta to [-PI, PI]
        while self.pose.theta > PI {
            self.pose.theta -= 2.0 * PI;
        }
        while self.pose.theta < -PI {
            self.pose.theta += 2.0 * PI;
        }

        self.pose
    }

    /// Get current attitude estimate
    pub fn attitude(&self) -> Attitude {
        self.attitude
    }

    /// Get current pose estimate
    pub fn pose(&self) -> Pose2D {
        self.pose
    }

    /// Reset pose to origin
    pub fn reset_pose(&mut self) {
        self.pose = Pose2D::default();
        self.last_encoder = EncoderData::default();
    }

    /// Reset attitude filters
    pub fn reset_attitude(&mut self) {
        self.attitude = Attitude::default();
        self.roll_filter.reset();
        self.pitch_filter.reset();
    }
}

impl Default for SensorServer {
    fn default() -> Self {
        Self::new(ImuCalibration::default(), OdometryConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_server() -> SensorServer {
        SensorServer::default()
    }

    // === IMU Calibration Tests ===

    #[test]
    fn imu_calibration_default_reasonable() {
        let cal = ImuCalibration::default();
        // At rest, 1g = ~16384 counts should give ~9.81 m/s²
        assert!((cal.accel_scale * 16384.0 - GRAVITY).abs() < 0.1);
    }

    #[test]
    fn calibrate_imu_zero_raw_gives_zero() {
        let server = make_server();
        let raw = ImuRaw::default();
        let imu = server.calibrate_imu(raw);

        assert_eq!(imu.accel_x, 0.0);
        assert_eq!(imu.accel_y, 0.0);
        assert_eq!(imu.accel_z, 0.0);
        assert_eq!(imu.gyro_x, 0.0);
    }

    #[test]
    fn calibrate_imu_1g_z_axis() {
        let server = make_server();
        let raw = ImuRaw {
            accel_z: 16384,  // 1g in default scale
            ..Default::default()
        };
        let imu = server.calibrate_imu(raw);

        // Should be close to 9.81 m/s²
        assert!((imu.accel_z - GRAVITY).abs() < 0.1);
    }

    #[test]
    fn calibrate_imu_applies_offsets() {
        let mut server = make_server();
        server.imu_cal.accel_offset = (100, 0, 0);

        let raw = ImuRaw {
            accel_x: 100,  // Equals offset, should give 0
            ..Default::default()
        };
        let imu = server.calibrate_imu(raw);

        assert_eq!(imu.accel_x, 0.0);
    }

    // === Attitude Estimation Tests ===

    #[test]
    fn attitude_starts_at_zero() {
        let server = make_server();
        let att = server.attitude();
        assert_eq!(att.roll, 0.0);
        assert_eq!(att.pitch, 0.0);
        assert_eq!(att.yaw, 0.0);
    }

    #[test]
    fn process_imu_updates_attitude() {
        let mut server = make_server();

        // Simulate tilted IMU (~30° roll)
        // atan2(8192, 14189) ≈ 30°
        let raw = ImuRaw {
            accel_y: 8192,
            accel_z: 14189,
            ..Default::default()
        };

        // Run a few iterations to let filter converge
        for _ in 0..10 {
            server.process_imu(raw, 0.01);
        }
        let att = server.attitude();

        // Should have approximately 30° (0.52 rad) roll
        // With alpha=0.98 complementary filter, it takes time to converge
        // Just check it's non-zero and in right direction
        assert!(att.roll > 0.05, "Roll should be positive, got {}", att.roll);
    }

    #[test]
    fn reset_attitude_clears_state() {
        let mut server = make_server();

        // Process some data
        let raw = ImuRaw {
            accel_y: 8192,
            accel_z: 14000,
            gyro_z: 1000,
            ..Default::default()
        };
        server.process_imu(raw, 0.1);

        server.reset_attitude();
        let att = server.attitude();

        assert_eq!(att.roll, 0.0);
        assert_eq!(att.pitch, 0.0);
        assert_eq!(att.yaw, 0.0);
    }

    // === Encoder/Odometry Tests ===

    #[test]
    fn pose_starts_at_origin() {
        let server = make_server();
        let pose = server.pose();
        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn forward_motion_increases_x() {
        let mut server = make_server();

        // Both wheels move forward equally
        let encoder = EncoderData {
            left_position: 100,
            right_position: 100,
            ..Default::default()
        };

        server.process_encoders(encoder, 0.01);
        let pose = server.pose();

        assert!(pose.x > 0.0, "Forward motion should increase X");
        assert!((pose.y).abs() < 0.001, "No lateral motion");
        assert!((pose.theta).abs() < 0.001, "No rotation");
    }

    #[test]
    fn turn_in_place_changes_theta() {
        let mut server = make_server();

        // Wheels move in opposite directions (turn in place)
        let encoder = EncoderData {
            left_position: -100,
            right_position: 100,
            ..Default::default()
        };

        server.process_encoders(encoder, 0.01);
        let pose = server.pose();

        assert!((pose.x).abs() < 0.001, "No forward motion");
        assert!(pose.theta.abs() > 0.01, "Should have rotated");
    }

    #[test]
    fn arc_motion_changes_all() {
        let mut server = make_server();

        // Right wheel faster = left turn
        let encoder = EncoderData {
            left_position: 50,
            right_position: 100,
            ..Default::default()
        };

        server.process_encoders(encoder, 0.01);
        let pose = server.pose();

        // Should have moved forward and turned left (positive theta)
        assert!(pose.x > 0.0);
        assert!(pose.theta > 0.0);
    }

    #[test]
    fn reset_pose_clears_position() {
        let mut server = make_server();

        let encoder = EncoderData {
            left_position: 100,
            right_position: 100,
            ..Default::default()
        };
        server.process_encoders(encoder, 0.01);

        server.reset_pose();
        let pose = server.pose();

        assert_eq!(pose.x, 0.0);
        assert_eq!(pose.y, 0.0);
        assert_eq!(pose.theta, 0.0);
    }

    #[test]
    fn theta_normalizes_to_pi() {
        let mut server = make_server();

        // Spin multiple times
        for _ in 0..100 {
            let encoder = EncoderData {
                left_position: -1000,
                right_position: 1000,
                ..Default::default()
            };
            server.process_encoders(encoder, 0.01);
            // Reset for next iteration
            server.last_encoder = EncoderData::default();
        }

        let pose = server.pose();
        assert!(pose.theta >= -PI && pose.theta <= PI);
    }

    // === Configuration Tests ===

    #[test]
    fn odometry_config_default_reasonable() {
        let cfg = OdometryConfig::default();
        assert!(cfg.wheel_radius > 0.0);
        assert!(cfg.track_width > 0.0);
        assert!(cfg.counts_per_rev > 0);
    }

    #[test]
    fn default_creates_server() {
        let server = SensorServer::default();
        assert_eq!(server.pose().x, 0.0);
        assert_eq!(server.attitude().roll, 0.0);
    }

    // === Data Type Tests ===

    #[test]
    fn imu_raw_default_is_zero() {
        let raw = ImuRaw::default();
        assert_eq!(raw.accel_x, 0);
        assert_eq!(raw.gyro_z, 0);
    }

    #[test]
    fn encoder_data_comparable() {
        let enc1 = EncoderData { left_position: 100, right_position: 200, ..Default::default() };
        let enc2 = EncoderData { left_position: 100, right_position: 200, ..Default::default() };
        assert_eq!(enc1, enc2);
    }
}
