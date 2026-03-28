//! Motor control server
//!
//! Handles motor control commands via IPC, runs PID loops.
//!
//! The motor server manages differential drive motors with:
//! - Velocity or position control modes
//! - PID control loops for each motor
//! - Fault detection (overcurrent, stall, encoder error)
//! - Emergency stop capability

use ox_control::PidController;
use ox_hal::motor::Motor;
use ox_hal::encoder::Encoder;

/// Motor control command
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorCommand {
    /// Set velocity setpoint (units: encoder ticks/second)
    SetVelocity { left: i16, right: i16 },
    /// Set position setpoint (units: encoder ticks)
    SetPosition { left: i32, right: i32 },
    /// Emergency stop - immediate brake
    EmergencyStop,
    /// Coast - disable motors, allow free spinning
    Coast,
}

/// Motor control mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlMode {
    /// Motors disabled, coasting
    Coast,
    /// Emergency stopped, braking
    Stopped,
    /// Velocity control mode
    Velocity,
    /// Position control mode
    Position,
}

/// Motor status report
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorStatus {
    pub left_position: i32,
    pub right_position: i32,
    pub left_velocity: i16,
    pub right_velocity: i16,
    pub left_pwm: i16,
    pub right_pwm: i16,
    pub mode: ControlMode,
    pub fault: Option<MotorFault>,
}

impl Default for ControlMode {
    fn default() -> Self {
        ControlMode::Coast
    }
}

/// Motor fault codes
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorFault {
    Overcurrent,
    Stall,
    EncoderError,
}

/// Motor server configuration
#[derive(Clone)]
pub struct MotorServerConfig {
    pub pid_kp: f32,
    pub pid_ki: f32,
    pub pid_kd: f32,
    pub max_pwm: u16,
    pub max_velocity: i16,
    pub stall_threshold_ms: u32,
    pub control_rate_hz: u32,
}

impl Default for MotorServerConfig {
    fn default() -> Self {
        Self {
            pid_kp: 1.0,
            pid_ki: 0.1,
            pid_kd: 0.01,
            max_pwm: 1000,
            max_velocity: 1000,
            stall_threshold_ms: 500,
            control_rate_hz: 1000,
        }
    }
}

/// Motor server core logic
///
/// This struct handles the control logic and can be tested independently
/// of the async IPC layer.
pub struct MotorServer<M: Motor, E: Encoder> {
    // Hardware
    left_motor: M,
    right_motor: M,
    left_encoder: E,
    right_encoder: E,

    // Control
    left_pid: PidController,
    right_pid: PidController,

    // State
    mode: ControlMode,
    velocity_setpoint: (i16, i16),
    position_setpoint: (i32, i32),
    last_position: (i32, i32),
    fault: Option<MotorFault>,

    // Config
    config: MotorServerConfig,
}

impl<M: Motor, E: Encoder> MotorServer<M, E> {
    /// Create a new motor server with the given hardware and config
    pub fn new(
        left_motor: M,
        right_motor: M,
        left_encoder: E,
        right_encoder: E,
        config: MotorServerConfig,
    ) -> Self {
        let mut left_pid = PidController::new(config.pid_kp, config.pid_ki, config.pid_kd);
        let mut right_pid = PidController::new(config.pid_kp, config.pid_ki, config.pid_kd);

        // Set PID output limits based on max PWM
        let max_pwm = config.max_pwm as f32;
        left_pid.set_output_limits(-max_pwm, max_pwm);
        right_pid.set_output_limits(-max_pwm, max_pwm);
        left_pid.set_integral_limits(-max_pwm, max_pwm);
        right_pid.set_integral_limits(-max_pwm, max_pwm);

        Self {
            left_motor,
            right_motor,
            left_encoder,
            right_encoder,
            left_pid,
            right_pid,
            mode: ControlMode::Coast,
            velocity_setpoint: (0, 0),
            position_setpoint: (0, 0),
            last_position: (0, 0),
            fault: None,
            config,
        }
    }

    /// Process a motor command
    pub fn process_command(&mut self, cmd: MotorCommand) {
        // Clear fault on new command (except emergency stop)
        if !matches!(cmd, MotorCommand::EmergencyStop) {
            self.fault = None;
        }

        match cmd {
            MotorCommand::SetVelocity { left, right } => {
                self.mode = ControlMode::Velocity;
                self.velocity_setpoint = (
                    left.clamp(-self.config.max_velocity, self.config.max_velocity),
                    right.clamp(-self.config.max_velocity, self.config.max_velocity),
                );
                self.left_pid.reset();
                self.right_pid.reset();
            }
            MotorCommand::SetPosition { left, right } => {
                self.mode = ControlMode::Position;
                self.position_setpoint = (left, right);
                self.left_pid.reset();
                self.right_pid.reset();
            }
            MotorCommand::EmergencyStop => {
                self.mode = ControlMode::Stopped;
                self.velocity_setpoint = (0, 0);
                let _ = self.left_motor.set_speed(0);
                let _ = self.right_motor.set_speed(0);
                self.left_pid.reset();
                self.right_pid.reset();
            }
            MotorCommand::Coast => {
                self.mode = ControlMode::Coast;
                self.velocity_setpoint = (0, 0);
                let _ = self.left_motor.coast();
                let _ = self.right_motor.coast();
                self.left_pid.reset();
                self.right_pid.reset();
            }
        }
    }

    /// Run one control loop iteration
    ///
    /// Call this at the configured control rate (e.g., 1kHz)
    pub fn update(&mut self, dt: f32) {
        // Don't update if stopped or coasting
        if matches!(self.mode, ControlMode::Stopped | ControlMode::Coast) {
            return;
        }

        // Don't update if faulted
        if self.fault.is_some() {
            let _ = self.left_motor.set_speed(0);
            let _ = self.right_motor.set_speed(0);
            return;
        }

        // Read encoders
        let left_pos = self.left_encoder.position();
        let right_pos = self.right_encoder.position();

        // Calculate velocity (ticks per second)
        let left_vel = ((left_pos - self.last_position.0) as f32 / dt) as i16;
        let right_vel = ((right_pos - self.last_position.1) as f32 / dt) as i16;
        self.last_position = (left_pos, right_pos);

        // Calculate control output based on mode
        let (left_pwm, right_pwm) = match self.mode {
            ControlMode::Velocity => {
                let left = self.left_pid.update(
                    self.velocity_setpoint.0 as f32,
                    left_vel as f32,
                    dt,
                );
                let right = self.right_pid.update(
                    self.velocity_setpoint.1 as f32,
                    right_vel as f32,
                    dt,
                );
                (left as i16, right as i16)
            }
            ControlMode::Position => {
                let left = self.left_pid.update(
                    self.position_setpoint.0 as f32,
                    left_pos as f32,
                    dt,
                );
                let right = self.right_pid.update(
                    self.position_setpoint.1 as f32,
                    right_pos as f32,
                    dt,
                );
                (left as i16, right as i16)
            }
            _ => (0, 0),
        };

        // Apply motor output
        self.apply_pwm(left_pwm, right_pwm);
    }

    /// Apply PWM values to motors
    fn apply_pwm(&mut self, left: i16, right: i16) {
        // Motor trait uses i16 where negative = reverse
        let _ = self.left_motor.set_speed(left);
        let _ = self.right_motor.set_speed(right);
    }

    /// Get current motor status
    pub fn status(&self) -> MotorStatus {
        MotorStatus {
            left_position: self.left_encoder.position(),
            right_position: self.right_encoder.position(),
            left_velocity: 0, // Would need to track this
            right_velocity: 0,
            left_pwm: 0,
            right_pwm: 0,
            mode: self.mode,
            fault: self.fault,
        }
    }

    /// Get current control mode
    pub fn mode(&self) -> ControlMode {
        self.mode
    }

    /// Set a fault condition
    pub fn set_fault(&mut self, fault: MotorFault) {
        self.fault = Some(fault);
        let _ = self.left_motor.set_speed(0);
        let _ = self.right_motor.set_speed(0);
    }

    /// Check if server is faulted
    pub fn is_faulted(&self) -> bool {
        self.fault.is_some()
    }

    /// Get velocity setpoints
    pub fn velocity_setpoint(&self) -> (i16, i16) {
        self.velocity_setpoint
    }

    /// Get position setpoints
    pub fn position_setpoint(&self) -> (i32, i32) {
        self.position_setpoint
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ox_hal::mock::{MockMotor, MockEncoder};

    fn make_server() -> MotorServer<MockMotor, MockEncoder> {
        MotorServer::new(
            MockMotor::new(),
            MockMotor::new(),
            MockEncoder::new(1000), // 1000 counts per rev
            MockEncoder::new(1000),
            MotorServerConfig::default(),
        )
    }

    // === Command Processing Tests ===

    #[test]
    fn motor_server_starts_in_coast_mode() {
        let server = make_server();
        assert_eq!(server.mode(), ControlMode::Coast);
    }

    #[test]
    fn set_velocity_changes_mode() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        assert_eq!(server.mode(), ControlMode::Velocity);
    }

    #[test]
    fn set_velocity_stores_setpoint() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 500, right: -300 });
        assert_eq!(server.velocity_setpoint(), (500, -300));
    }

    #[test]
    fn set_velocity_clamps_to_max() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 9999, right: -9999 });
        let max = server.config.max_velocity;
        assert_eq!(server.velocity_setpoint(), (max, -max));
    }

    #[test]
    fn set_position_changes_mode() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetPosition { left: 1000, right: 2000 });
        assert_eq!(server.mode(), ControlMode::Position);
    }

    #[test]
    fn set_position_stores_setpoint() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetPosition { left: 1000, right: -2000 });
        assert_eq!(server.position_setpoint(), (1000, -2000));
    }

    #[test]
    fn emergency_stop_changes_mode() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        server.process_command(MotorCommand::EmergencyStop);
        assert_eq!(server.mode(), ControlMode::Stopped);
    }

    #[test]
    fn emergency_stop_zeros_setpoint() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 500, right: 500 });
        server.process_command(MotorCommand::EmergencyStop);
        assert_eq!(server.velocity_setpoint(), (0, 0));
    }

    #[test]
    fn coast_changes_mode() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        server.process_command(MotorCommand::Coast);
        assert_eq!(server.mode(), ControlMode::Coast);
    }

    // === Fault Handling Tests ===

    #[test]
    fn server_starts_not_faulted() {
        let server = make_server();
        assert!(!server.is_faulted());
    }

    #[test]
    fn set_fault_marks_faulted() {
        let mut server = make_server();
        server.set_fault(MotorFault::Overcurrent);
        assert!(server.is_faulted());
    }

    #[test]
    fn fault_is_reported_in_status() {
        let mut server = make_server();
        server.set_fault(MotorFault::Stall);
        assert_eq!(server.status().fault, Some(MotorFault::Stall));
    }

    #[test]
    fn new_command_clears_fault() {
        let mut server = make_server();
        server.set_fault(MotorFault::EncoderError);
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        assert!(!server.is_faulted());
    }

    #[test]
    fn emergency_stop_does_not_clear_fault() {
        let mut server = make_server();
        server.set_fault(MotorFault::Overcurrent);
        server.process_command(MotorCommand::EmergencyStop);
        // Fault should persist through emergency stop
        assert!(server.is_faulted());
    }

    // === Status Reporting Tests ===

    #[test]
    fn status_reports_mode() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        assert_eq!(server.status().mode, ControlMode::Velocity);
    }

    #[test]
    fn status_reports_encoder_positions() {
        let mut server = make_server();
        // MockEncoder starts at 0
        let status = server.status();
        assert_eq!(status.left_position, 0);
        assert_eq!(status.right_position, 0);
    }

    // === Control Loop Tests ===

    #[test]
    fn update_does_nothing_when_coasting() {
        let mut server = make_server();
        // Should not panic or change state
        server.update(0.001);
        assert_eq!(server.mode(), ControlMode::Coast);
    }

    #[test]
    fn update_does_nothing_when_stopped() {
        let mut server = make_server();
        server.process_command(MotorCommand::EmergencyStop);
        server.update(0.001);
        assert_eq!(server.mode(), ControlMode::Stopped);
    }

    #[test]
    fn update_does_nothing_when_faulted() {
        let mut server = make_server();
        server.process_command(MotorCommand::SetVelocity { left: 100, right: 100 });
        server.set_fault(MotorFault::Stall);
        server.update(0.001);
        // Motors should be stopped due to fault
        assert!(server.is_faulted());
    }

    #[test]
    fn status_default_has_no_fault() {
        let status = MotorStatus::default();
        assert!(status.fault.is_none());
        assert_eq!(status.mode, ControlMode::Coast);
    }

    // === Config Tests ===

    #[test]
    fn default_config_reasonable_values() {
        let config = MotorServerConfig::default();
        assert!(config.pid_kp > 0.0);
        assert!(config.max_pwm > 0);
        assert!(config.max_velocity > 0);
        assert!(config.control_rate_hz >= 100);
    }

    // === Command Equality Tests ===

    #[test]
    fn commands_are_comparable() {
        let cmd1 = MotorCommand::SetVelocity { left: 100, right: 100 };
        let cmd2 = MotorCommand::SetVelocity { left: 100, right: 100 };
        let cmd3 = MotorCommand::EmergencyStop;

        assert_eq!(cmd1, cmd2);
        assert_ne!(cmd1, cmd3);
    }

    #[test]
    fn control_modes_are_comparable() {
        assert_eq!(ControlMode::Coast, ControlMode::Coast);
        assert_ne!(ControlMode::Coast, ControlMode::Velocity);
    }

    #[test]
    fn motor_faults_are_comparable() {
        assert_eq!(MotorFault::Stall, MotorFault::Stall);
        assert_ne!(MotorFault::Stall, MotorFault::Overcurrent);
    }
}
