//! Motor control server
//!
//! Handles motor control commands via IPC, runs PID loops.

// These imports will be used when the server task is implemented
#[allow(unused_imports)]
use ox_kernel::ipc::Stream;
#[allow(unused_imports)]
use ox_kernel::cap::{Cap, PwmChannel, GpioPin};
#[allow(unused_imports)]
use ox_control::PidController;

/// Motor control command
#[derive(Debug, Clone, Copy)]
pub enum MotorCommand {
    /// Set velocity setpoint
    SetVelocity { left: i16, right: i16 },
    /// Set position setpoint
    SetPosition { left: i32, right: i32 },
    /// Emergency stop
    EmergencyStop,
    /// Coast (disable motors)
    Coast,
}

/// Motor status report
#[derive(Debug, Clone, Copy)]
pub struct MotorStatus {
    pub left_position: i32,
    pub right_position: i32,
    pub left_velocity: i16,
    pub right_velocity: i16,
    pub fault: Option<MotorFault>,
}

/// Motor fault codes
#[derive(Debug, Clone, Copy)]
pub enum MotorFault {
    Overcurrent,
    Stall,
    EncoderError,
}

/// Motor server configuration
pub struct MotorServerConfig {
    pub pid_kp: f32,
    pub pid_ki: f32,
    pub pid_kd: f32,
    pub max_velocity: i16,
    pub control_rate_hz: u32,
}

impl Default for MotorServerConfig {
    fn default() -> Self {
        Self {
            pid_kp: 1.0,
            pid_ki: 0.1,
            pid_kd: 0.01,
            max_velocity: 1000,
            control_rate_hz: 1000,
        }
    }
}

// Motor server task will be implemented when integrating with Embassy executor
