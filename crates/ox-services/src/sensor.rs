//! Sensor fusion server
//!
//! Handles IMU, encoders, and other sensors.

// This import will be used when the server task is implemented
#[allow(unused_imports)]
use ox_kernel::ipc::Stream;

/// IMU data
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Accelerometer X (m/s^2)
    pub accel_x: f32,
    /// Accelerometer Y (m/s^2)
    pub accel_y: f32,
    /// Accelerometer Z (m/s^2)
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
    /// Roll angle (radians)
    pub roll: f32,
    /// Pitch angle (radians)
    pub pitch: f32,
    /// Yaw angle (radians)
    pub yaw: f32,
}

/// Encoder readings
#[derive(Debug, Clone, Copy, Default)]
pub struct EncoderData {
    pub left_position: i32,
    pub right_position: i32,
    pub left_velocity: i16,
    pub right_velocity: i16,
}

// Sensor server task will be implemented when integrating with Embassy executor
