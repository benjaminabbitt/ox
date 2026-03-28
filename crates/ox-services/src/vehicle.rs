//! Vehicle State Machine
//!
//! Central state machine for vehicle mode control and safety.
//!
//! Features:
//! - Mode management (Disarmed, Manual, Armed, Auto, RTL, Emergency)
//! - Arming checks (GPS, compass, RC, mission, home)
//! - Safe state transitions with guards
//! - Emergency stop handling
//!
//! Architecture:
//! ```text
//! RC Task ──> VehicleCommand ──> VehicleStateMachine
//!                                       │
//! NavServer ─────────────────────────────┤
//!                                       │
//! GPS/Compass ───────────────────────────┘
//! ```

// ============================================================================
// Vehicle Mode
// ============================================================================

/// Vehicle operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum VehicleMode {
    /// Motors disabled, safe state (default)
    #[default]
    Disarmed = 0,
    /// RC direct control (passthrough)
    Manual = 1,
    /// Armed and ready for autonomous operation
    Armed = 2,
    /// Autonomous navigation active
    Auto = 3,
    /// Returning to launch position
    Rtl = 4,
    /// Emergency/failsafe state
    Emergency = 5,
}

impl VehicleMode {
    /// Check if motors can be commanded in this mode
    pub fn motors_enabled(&self) -> bool {
        !matches!(self, VehicleMode::Disarmed | VehicleMode::Emergency)
    }

    /// Check if this is an autonomous mode
    pub fn is_autonomous(&self) -> bool {
        matches!(self, VehicleMode::Auto | VehicleMode::Rtl)
    }
}

// ============================================================================
// Arming Checks
// ============================================================================

/// Pre-arm safety checks
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmingChecks {
    /// GPS has valid 3D fix
    pub gps_fix: bool,
    /// Compass is calibrated
    pub compass_calibrated: bool,
    /// Mission is loaded (for auto mode)
    pub mission_loaded: bool,
    /// Home position is set (for RTL)
    pub home_set: bool,
    /// RC receiver connected
    pub rc_connected: bool,
    /// No active faults/errors
    pub no_faults: bool,
}

impl ArmingChecks {
    /// Create new arming checks with all failed
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if basic arming is allowed (Manual/Armed modes)
    pub fn can_arm(&self) -> bool {
        self.rc_connected && self.no_faults
    }

    /// Check if autonomous mode is allowed
    pub fn can_auto(&self) -> bool {
        self.can_arm() && self.gps_fix && self.compass_calibrated && self.mission_loaded
    }

    /// Check if RTL is allowed
    pub fn can_rtl(&self) -> bool {
        self.can_arm() && self.gps_fix && self.compass_calibrated && self.home_set
    }

    /// Count how many checks are passing
    pub fn passing_count(&self) -> u8 {
        let mut count = 0;
        if self.gps_fix {
            count += 1;
        }
        if self.compass_calibrated {
            count += 1;
        }
        if self.mission_loaded {
            count += 1;
        }
        if self.home_set {
            count += 1;
        }
        if self.rc_connected {
            count += 1;
        }
        if self.no_faults {
            count += 1;
        }
        count
    }

    /// Total number of checks
    pub const fn total_checks() -> u8 {
        6
    }
}

// ============================================================================
// Vehicle Commands
// ============================================================================

/// Commands to vehicle state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleCommand {
    /// Request arming (Disarmed -> Armed)
    Arm,
    /// Request disarming (Any -> Disarmed)
    Disarm,
    /// Switch to manual RC control
    Manual,
    /// Start autonomous navigation (Armed -> Auto)
    StartAuto,
    /// Stop autonomous, return to Armed
    StopAuto,
    /// Return to launch (Armed/Auto -> RTL)
    Rtl,
    /// Emergency stop (Any -> Emergency)
    EmergencyStop,
    /// Clear emergency state (requires disarm first)
    ClearEmergency,
}

// ============================================================================
// Transition Result
// ============================================================================

/// Result of a state transition attempt
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionResult {
    /// Transition succeeded
    Ok,
    /// Transition denied - not armed
    NotArmed,
    /// Transition denied - arming checks failed
    ArmingChecksFailed,
    /// Transition denied - no mission loaded
    NoMission,
    /// Transition denied - no home position
    NoHome,
    /// Transition denied - invalid from current state
    InvalidTransition,
    /// Transition denied - in emergency state
    InEmergency,
}

impl TransitionResult {
    /// Check if transition was successful
    pub fn is_ok(&self) -> bool {
        matches!(self, TransitionResult::Ok)
    }
}

// ============================================================================
// Vehicle State Machine
// ============================================================================

/// Vehicle state machine for mode management
pub struct VehicleStateMachine {
    /// Current vehicle mode
    mode: VehicleMode,
    /// Current arming checks status
    checks: ArmingChecks,
    /// Timestamp of last mode change (ms)
    last_mode_change_ms: u32,
    /// Time in current mode (ms)
    time_in_mode_ms: u32,
}

impl VehicleStateMachine {
    /// Create a new vehicle state machine in Disarmed state
    pub fn new() -> Self {
        Self {
            mode: VehicleMode::Disarmed,
            checks: ArmingChecks::new(),
            last_mode_change_ms: 0,
            time_in_mode_ms: 0,
        }
    }

    /// Get current mode
    pub fn mode(&self) -> VehicleMode {
        self.mode
    }

    /// Get current arming checks
    pub fn checks(&self) -> &ArmingChecks {
        &self.checks
    }

    /// Get time in current mode (ms)
    pub fn time_in_mode_ms(&self) -> u32 {
        self.time_in_mode_ms
    }

    /// Update arming checks from external sources
    pub fn update_checks(&mut self, checks: ArmingChecks) {
        self.checks = checks;
    }

    /// Update individual check - GPS fix
    pub fn set_gps_fix(&mut self, valid: bool) {
        self.checks.gps_fix = valid;
    }

    /// Update individual check - compass calibrated
    pub fn set_compass_calibrated(&mut self, calibrated: bool) {
        self.checks.compass_calibrated = calibrated;
    }

    /// Update individual check - mission loaded
    pub fn set_mission_loaded(&mut self, loaded: bool) {
        self.checks.mission_loaded = loaded;
    }

    /// Update individual check - home set
    pub fn set_home_set(&mut self, set: bool) {
        self.checks.home_set = set;
    }

    /// Update individual check - RC connected
    pub fn set_rc_connected(&mut self, connected: bool) {
        self.checks.rc_connected = connected;
    }

    /// Update individual check - no faults
    pub fn set_no_faults(&mut self, ok: bool) {
        self.checks.no_faults = ok;
    }

    /// Update timing (call once per cycle)
    pub fn update(&mut self, elapsed_ms: u32) {
        self.time_in_mode_ms = self.time_in_mode_ms.saturating_add(elapsed_ms);
    }

    /// Process a vehicle command
    pub fn process_command(&mut self, cmd: VehicleCommand) -> TransitionResult {
        match cmd {
            VehicleCommand::Arm => self.try_arm(),
            VehicleCommand::Disarm => self.try_disarm(),
            VehicleCommand::Manual => self.try_manual(),
            VehicleCommand::StartAuto => self.try_start_auto(),
            VehicleCommand::StopAuto => self.try_stop_auto(),
            VehicleCommand::Rtl => self.try_rtl(),
            VehicleCommand::EmergencyStop => self.emergency_stop(),
            VehicleCommand::ClearEmergency => self.try_clear_emergency(),
        }
    }

    /// Try to arm the vehicle
    fn try_arm(&mut self) -> TransitionResult {
        if self.mode == VehicleMode::Emergency {
            return TransitionResult::InEmergency;
        }

        if self.mode != VehicleMode::Disarmed {
            return TransitionResult::InvalidTransition;
        }

        if !self.checks.can_arm() {
            return TransitionResult::ArmingChecksFailed;
        }

        self.set_mode(VehicleMode::Armed);
        TransitionResult::Ok
    }

    /// Try to disarm the vehicle
    fn try_disarm(&mut self) -> TransitionResult {
        // Can always disarm (safety)
        self.set_mode(VehicleMode::Disarmed);
        TransitionResult::Ok
    }

    /// Try to switch to manual mode
    fn try_manual(&mut self) -> TransitionResult {
        if self.mode == VehicleMode::Emergency {
            return TransitionResult::InEmergency;
        }

        if self.mode == VehicleMode::Disarmed {
            return TransitionResult::NotArmed;
        }

        if !self.checks.can_arm() {
            return TransitionResult::ArmingChecksFailed;
        }

        self.set_mode(VehicleMode::Manual);
        TransitionResult::Ok
    }

    /// Try to start autonomous navigation
    fn try_start_auto(&mut self) -> TransitionResult {
        if self.mode == VehicleMode::Emergency {
            return TransitionResult::InEmergency;
        }

        if self.mode == VehicleMode::Disarmed {
            return TransitionResult::NotArmed;
        }

        if !self.checks.can_auto() {
            if !self.checks.mission_loaded {
                return TransitionResult::NoMission;
            }
            return TransitionResult::ArmingChecksFailed;
        }

        self.set_mode(VehicleMode::Auto);
        TransitionResult::Ok
    }

    /// Stop autonomous navigation
    fn try_stop_auto(&mut self) -> TransitionResult {
        if self.mode == VehicleMode::Emergency {
            return TransitionResult::InEmergency;
        }

        if self.mode != VehicleMode::Auto && self.mode != VehicleMode::Rtl {
            return TransitionResult::InvalidTransition;
        }

        self.set_mode(VehicleMode::Armed);
        TransitionResult::Ok
    }

    /// Try to initiate RTL
    fn try_rtl(&mut self) -> TransitionResult {
        if self.mode == VehicleMode::Emergency {
            return TransitionResult::InEmergency;
        }

        if self.mode == VehicleMode::Disarmed {
            return TransitionResult::NotArmed;
        }

        if !self.checks.can_rtl() {
            if !self.checks.home_set {
                return TransitionResult::NoHome;
            }
            return TransitionResult::ArmingChecksFailed;
        }

        self.set_mode(VehicleMode::Rtl);
        TransitionResult::Ok
    }

    /// Emergency stop - always succeeds
    fn emergency_stop(&mut self) -> TransitionResult {
        self.set_mode(VehicleMode::Emergency);
        TransitionResult::Ok
    }

    /// Try to clear emergency state
    fn try_clear_emergency(&mut self) -> TransitionResult {
        if self.mode != VehicleMode::Emergency {
            return TransitionResult::InvalidTransition;
        }

        // Must go through Disarmed
        self.set_mode(VehicleMode::Disarmed);
        TransitionResult::Ok
    }

    /// Set mode and reset timing
    fn set_mode(&mut self, mode: VehicleMode) {
        self.mode = mode;
        self.time_in_mode_ms = 0;
    }
}

impl Default for VehicleStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_passing_checks() -> ArmingChecks {
        ArmingChecks {
            gps_fix: true,
            compass_calibrated: true,
            mission_loaded: true,
            home_set: true,
            rc_connected: true,
            no_faults: true,
        }
    }

    // ========== Initial State Tests ==========

    #[test]
    fn starts_disarmed() {
        let vsm = VehicleStateMachine::new();
        assert_eq!(vsm.mode(), VehicleMode::Disarmed);
    }

    #[test]
    fn disarmed_motors_disabled() {
        assert!(!VehicleMode::Disarmed.motors_enabled());
    }

    #[test]
    fn armed_motors_enabled() {
        assert!(VehicleMode::Armed.motors_enabled());
    }

    #[test]
    fn emergency_motors_disabled() {
        assert!(!VehicleMode::Emergency.motors_enabled());
    }

    // ========== Arming Tests ==========

    #[test]
    fn arm_denied_without_rc() {
        let mut vsm = VehicleStateMachine::new();
        vsm.set_no_faults(true);
        // RC not connected
        let result = vsm.process_command(VehicleCommand::Arm);
        assert_eq!(result, TransitionResult::ArmingChecksFailed);
        assert_eq!(vsm.mode(), VehicleMode::Disarmed);
    }

    #[test]
    fn arm_denied_with_faults() {
        let mut vsm = VehicleStateMachine::new();
        vsm.set_rc_connected(true);
        // Faults present (no_faults = false)
        let result = vsm.process_command(VehicleCommand::Arm);
        assert_eq!(result, TransitionResult::ArmingChecksFailed);
    }

    #[test]
    fn arm_succeeds_with_basic_checks() {
        let mut vsm = VehicleStateMachine::new();
        vsm.set_rc_connected(true);
        vsm.set_no_faults(true);

        let result = vsm.process_command(VehicleCommand::Arm);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Armed);
    }

    #[test]
    fn arm_denied_when_already_armed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.set_rc_connected(true);
        vsm.set_no_faults(true);
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::Arm);
        assert_eq!(result, TransitionResult::InvalidTransition);
    }

    // ========== Disarm Tests ==========

    #[test]
    fn disarm_always_succeeds() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);
        vsm.process_command(VehicleCommand::StartAuto);

        let result = vsm.process_command(VehicleCommand::Disarm);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Disarmed);
    }

    #[test]
    fn disarm_from_emergency() {
        let mut vsm = VehicleStateMachine::new();
        vsm.process_command(VehicleCommand::EmergencyStop);

        let result = vsm.process_command(VehicleCommand::Disarm);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Disarmed);
    }

    // ========== Auto Mode Tests ==========

    #[test]
    fn auto_denied_without_gps() {
        let mut vsm = VehicleStateMachine::new();
        let mut checks = make_passing_checks();
        checks.gps_fix = false;
        vsm.update_checks(checks);
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::StartAuto);
        assert_eq!(result, TransitionResult::ArmingChecksFailed);
    }

    #[test]
    fn auto_denied_without_mission() {
        let mut vsm = VehicleStateMachine::new();
        let mut checks = make_passing_checks();
        checks.mission_loaded = false;
        vsm.update_checks(checks);
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::StartAuto);
        assert_eq!(result, TransitionResult::NoMission);
    }

    #[test]
    fn auto_denied_when_disarmed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        // Don't arm

        let result = vsm.process_command(VehicleCommand::StartAuto);
        assert_eq!(result, TransitionResult::NotArmed);
    }

    #[test]
    fn auto_succeeds_with_all_checks() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::StartAuto);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Auto);
    }

    #[test]
    fn stop_auto_returns_to_armed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);
        vsm.process_command(VehicleCommand::StartAuto);

        let result = vsm.process_command(VehicleCommand::StopAuto);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Armed);
    }

    // ========== RTL Tests ==========

    #[test]
    fn rtl_denied_without_home() {
        let mut vsm = VehicleStateMachine::new();
        let mut checks = make_passing_checks();
        checks.home_set = false;
        vsm.update_checks(checks);
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::Rtl);
        assert_eq!(result, TransitionResult::NoHome);
    }

    #[test]
    fn rtl_denied_when_disarmed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());

        let result = vsm.process_command(VehicleCommand::Rtl);
        assert_eq!(result, TransitionResult::NotArmed);
    }

    #[test]
    fn rtl_succeeds_from_armed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::Rtl);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Rtl);
    }

    #[test]
    fn rtl_succeeds_from_auto() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);
        vsm.process_command(VehicleCommand::StartAuto);

        let result = vsm.process_command(VehicleCommand::Rtl);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Rtl);
    }

    // ========== Emergency Tests ==========

    #[test]
    fn emergency_from_any_state() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);
        vsm.process_command(VehicleCommand::StartAuto);

        let result = vsm.process_command(VehicleCommand::EmergencyStop);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Emergency);
    }

    #[test]
    fn emergency_blocks_arm() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::EmergencyStop);

        let result = vsm.process_command(VehicleCommand::Arm);
        assert_eq!(result, TransitionResult::InEmergency);
    }

    #[test]
    fn emergency_blocks_auto() {
        let mut vsm = VehicleStateMachine::new();
        vsm.process_command(VehicleCommand::EmergencyStop);

        let result = vsm.process_command(VehicleCommand::StartAuto);
        assert_eq!(result, TransitionResult::InEmergency);
    }

    #[test]
    fn clear_emergency_to_disarmed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.process_command(VehicleCommand::EmergencyStop);

        let result = vsm.process_command(VehicleCommand::ClearEmergency);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Disarmed);
    }

    // ========== Manual Mode Tests ==========

    #[test]
    fn manual_from_armed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());
        vsm.process_command(VehicleCommand::Arm);

        let result = vsm.process_command(VehicleCommand::Manual);
        assert_eq!(result, TransitionResult::Ok);
        assert_eq!(vsm.mode(), VehicleMode::Manual);
    }

    #[test]
    fn manual_denied_when_disarmed() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update_checks(make_passing_checks());

        let result = vsm.process_command(VehicleCommand::Manual);
        assert_eq!(result, TransitionResult::NotArmed);
    }

    // ========== Mode Property Tests ==========

    #[test]
    fn auto_is_autonomous() {
        assert!(VehicleMode::Auto.is_autonomous());
    }

    #[test]
    fn rtl_is_autonomous() {
        assert!(VehicleMode::Rtl.is_autonomous());
    }

    #[test]
    fn manual_is_not_autonomous() {
        assert!(!VehicleMode::Manual.is_autonomous());
    }

    // ========== Arming Checks Tests ==========

    #[test]
    fn arming_checks_count() {
        let checks = make_passing_checks();
        assert_eq!(checks.passing_count(), 6);
    }

    #[test]
    fn arming_checks_partial() {
        let mut checks = ArmingChecks::new();
        checks.gps_fix = true;
        checks.rc_connected = true;
        assert_eq!(checks.passing_count(), 2);
    }

    // ========== Timing Tests ==========

    #[test]
    fn time_in_mode_tracked() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update(100);
        vsm.update(200);
        assert_eq!(vsm.time_in_mode_ms(), 300);
    }

    #[test]
    fn time_resets_on_mode_change() {
        let mut vsm = VehicleStateMachine::new();
        vsm.update(100);
        vsm.set_rc_connected(true);
        vsm.set_no_faults(true);
        vsm.process_command(VehicleCommand::Arm);
        assert_eq!(vsm.time_in_mode_ms(), 0);
    }
}
