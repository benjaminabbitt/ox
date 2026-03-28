//! Navigation Server
//!
//! Autonomous waypoint navigation for ground vehicles.
//!
//! Features:
//! - Waypoint following with arrival detection
//! - Return to Launch (RTL) with automatic home capture
//! - Mission management with auto-RTL
//! - Haversine distance and bearing calculations
//! - Proportional steering control
//! - Speed ramping on approach
//! - Sensor timeout handling (GPS/compass)
//!
//! Architecture:
//! ```text
//! GPS (10Hz) ──────┐
//!                  ├──> NavServer ──> MotorCommand
//! Compass (50Hz) ──┘        │
//!                           └──> NavTelemetry
//! ```

use crate::compass::CompassData;
use crate::gps::GpsData;
use crate::motor::MotorCommand;

// ============================================================================
// Constants
// ============================================================================

/// Maximum waypoints in a mission
pub const MAX_WAYPOINTS: usize = 32;

/// Earth radius in meters (WGS84 mean radius)
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Degrees to radians conversion factor
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Radians to degrees conversion factor
const RAD_TO_DEG: f64 = 180.0 / core::f64::consts::PI;

/// Default home arrival radius in meters
const DEFAULT_HOME_RADIUS_M: f32 = 5.0;

// ============================================================================
// Home Position (RTL)
// ============================================================================

/// Source of home position
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum HomeSource {
    /// No home position set
    #[default]
    None = 0,
    /// Captured from first GPS fix
    FirstFix = 1,
    /// Set manually via command
    Manual = 2,
    /// Defined by mission
    Mission = 3,
}

/// Home position for Return-To-Launch
#[derive(Debug, Clone, Copy, Default)]
pub struct HomePosition {
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters (relative to launch)
    pub altitude: f32,
    /// Timestamp when home was captured (ms since boot)
    pub timestamp_ms: u32,
    /// How home was set
    pub source: HomeSource,
    /// Whether home is valid for navigation
    pub valid: bool,
}

impl HomePosition {
    /// Create a new valid home position
    pub fn new(latitude: f64, longitude: f64, altitude: f32, timestamp_ms: u32, source: HomeSource) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
            timestamp_ms,
            source,
            valid: true,
        }
    }

    /// Create home from GPS data
    pub fn from_gps(gps: &GpsData, timestamp_ms: u32, source: HomeSource) -> Self {
        Self {
            latitude: gps.latitude,
            longitude: gps.longitude,
            altitude: gps.altitude,
            timestamp_ms,
            source,
            valid: true,
        }
    }
}

// ============================================================================
// Navigation Math
// ============================================================================

/// Navigation math functions
pub mod nav_math {
    use super::*;

    /// Calculate distance between two GPS coordinates using Haversine formula
    ///
    /// # Arguments
    /// * `lat1`, `lon1` - First point (degrees)
    /// * `lat2`, `lon2` - Second point (degrees)
    ///
    /// # Returns
    /// Distance in meters
    pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f32 {
        let lat1_rad = lat1 * DEG_TO_RAD;
        let lat2_rad = lat2 * DEG_TO_RAD;
        let delta_lat = (lat2 - lat1) * DEG_TO_RAD;
        let delta_lon = (lon2 - lon1) * DEG_TO_RAD;

        let sin_dlat = libm::sin(delta_lat / 2.0);
        let sin_dlon = libm::sin(delta_lon / 2.0);

        let a = sin_dlat * sin_dlat
            + libm::cos(lat1_rad) * libm::cos(lat2_rad) * sin_dlon * sin_dlon;

        let c = 2.0 * libm::atan2(libm::sqrt(a), libm::sqrt(1.0 - a));

        (EARTH_RADIUS_M * c) as f32
    }

    /// Calculate initial bearing from point 1 to point 2
    ///
    /// # Arguments
    /// * `lat1`, `lon1` - Starting point (degrees)
    /// * `lat2`, `lon2` - Destination point (degrees)
    ///
    /// # Returns
    /// Bearing in degrees (0-360, 0 = North, 90 = East)
    pub fn calculate_bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f32 {
        let lat1_rad = lat1 * DEG_TO_RAD;
        let lat2_rad = lat2 * DEG_TO_RAD;
        let delta_lon = (lon2 - lon1) * DEG_TO_RAD;

        let x = libm::sin(delta_lon) * libm::cos(lat2_rad);
        let y = libm::cos(lat1_rad) * libm::sin(lat2_rad)
            - libm::sin(lat1_rad) * libm::cos(lat2_rad) * libm::cos(delta_lon);

        let bearing_rad = libm::atan2(x, y);
        let mut bearing_deg = (bearing_rad * RAD_TO_DEG) as f32;

        // Normalize to 0-360
        if bearing_deg < 0.0 {
            bearing_deg += 360.0;
        }

        bearing_deg
    }

    /// Calculate heading error (how much to turn)
    ///
    /// # Arguments
    /// * `current_heading` - Current heading in degrees (0-360)
    /// * `desired_heading` - Desired heading in degrees (0-360)
    ///
    /// # Returns
    /// Error in degrees (-180 to 180, positive = turn right/clockwise)
    pub fn heading_error(current_heading: f32, desired_heading: f32) -> f32 {
        let mut error = desired_heading - current_heading;

        // Normalize to -180..180
        while error > 180.0 {
            error -= 360.0;
        }
        while error < -180.0 {
            error += 360.0;
        }

        error
    }

    /// Calculate speed based on distance with linear ramp-down on approach
    ///
    /// # Arguments
    /// * `distance_m` - Distance to waypoint in meters
    /// * `approach_distance_m` - Distance at which to start slowing
    /// * `max_speed` - Maximum speed (0.0 to 1.0)
    /// * `min_speed` - Minimum speed (0.0 to 1.0)
    ///
    /// # Returns
    /// Speed factor (min_speed to max_speed)
    pub fn calculate_speed(
        distance_m: f32,
        approach_distance_m: f32,
        max_speed: f32,
        min_speed: f32,
    ) -> f32 {
        if distance_m >= approach_distance_m {
            max_speed
        } else if distance_m <= 0.0 {
            min_speed
        } else {
            // Linear interpolation
            let ratio = distance_m / approach_distance_m;
            min_speed + (max_speed - min_speed) * ratio
        }
    }

    /// Convert throttle and steering to differential drive motor commands
    ///
    /// # Arguments
    /// * `speed` - Forward speed (-1.0 to 1.0, positive = forward)
    /// * `steering` - Steering input (-1.0 to 1.0, positive = turn right)
    /// * `max_velocity` - Maximum motor velocity value
    ///
    /// # Returns
    /// (left_velocity, right_velocity) as i16
    pub fn differential_mix(speed: f32, steering: f32, max_velocity: i16) -> (i16, i16) {
        // For right turn: left wheel faster, right wheel slower
        let left = (speed + steering).clamp(-1.0, 1.0);
        let right = (speed - steering).clamp(-1.0, 1.0);

        let left_vel = (left * max_velocity as f32) as i16;
        let right_vel = (right * max_velocity as f32) as i16;

        (left_vel, right_vel)
    }
}

// ============================================================================
// Data Types
// ============================================================================

/// A geographic waypoint for navigation
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Waypoint {
    /// Latitude in degrees (positive = North)
    pub latitude: f64,
    /// Longitude in degrees (positive = East)
    pub longitude: f64,
    /// Arrival radius in meters (waypoint reached when within this distance)
    pub radius: f32,
}

impl Waypoint {
    /// Create a new waypoint with default arrival radius
    pub fn new(latitude: f64, longitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            radius: 0.0, // Use default from config
        }
    }

    /// Create a new waypoint with custom arrival radius
    pub fn with_radius(latitude: f64, longitude: f64, radius: f32) -> Self {
        Self {
            latitude,
            longitude,
            radius,
        }
    }
}

// ============================================================================
// Mission Management
// ============================================================================

/// Mission metadata
#[derive(Debug, Clone, Copy, Default)]
pub struct MissionMeta {
    /// Unique mission ID
    pub id: u16,
    /// Number of waypoints in mission
    pub count: u8,
    /// CRC16 checksum for validation
    pub checksum: u16,
}

impl MissionMeta {
    /// Create new mission metadata
    pub fn new(id: u16, count: u8) -> Self {
        Self {
            id,
            count,
            checksum: 0,
        }
    }
}

/// Mission behavior flags
#[derive(Debug, Clone, Copy, Default)]
pub struct MissionFlags {
    /// Automatically RTL after mission complete
    pub auto_rtl: bool,
    /// Loop mission continuously
    pub loop_mission: bool,
}

/// A complete mission with waypoints and metadata
#[derive(Debug, Clone)]
pub struct Mission {
    /// Mission metadata
    pub meta: MissionMeta,
    /// Mission behavior flags
    pub flags: MissionFlags,
    /// Optional mission-defined home position
    pub home: Option<HomePosition>,
    /// Waypoints in the mission
    pub waypoints: [Option<Waypoint>; MAX_WAYPOINTS],
}

impl Mission {
    /// Create a new empty mission
    pub fn new(id: u16) -> Self {
        Self {
            meta: MissionMeta::new(id, 0),
            flags: MissionFlags::default(),
            home: None,
            waypoints: [None; MAX_WAYPOINTS],
        }
    }

    /// Add a waypoint to the mission
    pub fn add_waypoint(&mut self, wp: Waypoint) -> bool {
        if (self.meta.count as usize) < MAX_WAYPOINTS {
            self.waypoints[self.meta.count as usize] = Some(wp);
            self.meta.count += 1;
            true
        } else {
            false
        }
    }

    /// Calculate checksum for the mission
    pub fn calculate_checksum(&self) -> u16 {
        let mut crc: u16 = 0xFFFF;

        // Include metadata
        crc = crc_update(crc, self.meta.id as u8);
        crc = crc_update(crc, (self.meta.id >> 8) as u8);
        crc = crc_update(crc, self.meta.count);

        // Include flags
        let flags_byte = (self.flags.auto_rtl as u8) | ((self.flags.loop_mission as u8) << 1);
        crc = crc_update(crc, flags_byte);

        // Include waypoints
        for i in 0..self.meta.count as usize {
            if let Some(wp) = self.waypoints[i] {
                // Hash lat/lon as fixed-point integers
                let lat_i = (wp.latitude * 1e7) as i32;
                let lon_i = (wp.longitude * 1e7) as i32;

                for byte in lat_i.to_le_bytes() {
                    crc = crc_update(crc, byte);
                }
                for byte in lon_i.to_le_bytes() {
                    crc = crc_update(crc, byte);
                }
            }
        }

        crc
    }

    /// Validate mission checksum
    pub fn validate(&self) -> bool {
        self.meta.checksum == self.calculate_checksum()
    }

    /// Finalize mission by calculating and setting checksum
    pub fn finalize(&mut self) {
        self.meta.checksum = self.calculate_checksum();
    }

    /// Get waypoint count
    pub fn waypoint_count(&self) -> u8 {
        self.meta.count
    }
}

impl Default for Mission {
    fn default() -> Self {
        Self::new(0)
    }
}

/// CRC-16 update function (CCITT polynomial)
fn crc_update(crc: u16, byte: u8) -> u16 {
    let mut x = crc;
    x ^= (byte as u16) << 8;
    for _ in 0..8 {
        if (x & 0x8000) != 0 {
            x = (x << 1) ^ 0x1021;
        } else {
            x <<= 1;
        }
    }
    x
}

/// Navigation state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum NavState {
    /// No mission loaded or mission cleared
    #[default]
    Idle = 0,
    /// Actively navigating to waypoints
    Active = 1,
    /// All waypoints reached, mission complete
    Arrived = 2,
    /// Waiting for valid GPS fix or compass calibration
    Hold = 3,
    /// Navigation paused by external command
    Paused = 4,
    /// Returning to home position (RTL)
    ReturningHome = 5,
    /// Arrived at home position
    HomeReached = 6,
}

/// Navigation server configuration
#[derive(Debug, Clone, Copy)]
pub struct NavConfig {
    /// Default waypoint arrival radius in meters
    pub default_radius_m: f32,
    /// Maximum forward speed (0.0 to 1.0, normalized)
    pub max_speed: f32,
    /// Minimum speed for slow approach (0.0 to 1.0)
    pub min_speed: f32,
    /// Distance at which to start slowing down (meters)
    pub approach_distance_m: f32,
    /// Steering proportional gain (heading error -> steering)
    pub steering_kp: f32,
    /// Steering deadband in degrees (no correction below this error)
    pub steering_deadband_deg: f32,
    /// Maximum steering output (-1.0 to 1.0)
    pub max_steering: f32,
    /// Maximum motor velocity value for differential_mix
    pub max_motor_velocity: i16,
    /// GPS timeout before entering Hold state (ms)
    pub gps_timeout_ms: u32,
    /// Compass timeout before entering Hold state (ms)
    pub compass_timeout_ms: u32,
    /// Maximum acceptable GPS HDOP (lower is better)
    pub max_hdop: f32,
    /// Require compass calibration for navigation
    pub require_compass_cal: bool,
    /// Home arrival radius in meters (RTL complete)
    pub home_radius_m: f32,
    /// Automatically capture first GPS fix as home
    pub auto_capture_home: bool,
    /// Automatically RTL after mission complete
    pub auto_rtl_on_complete: bool,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            default_radius_m: 3.0,
            max_speed: 0.5,
            min_speed: 0.15,
            approach_distance_m: 10.0,
            steering_kp: 0.02,
            steering_deadband_deg: 5.0,
            max_steering: 0.8,
            max_motor_velocity: 1000,
            gps_timeout_ms: 2000,
            compass_timeout_ms: 500,
            max_hdop: 5.0,
            require_compass_cal: true,
            home_radius_m: DEFAULT_HOME_RADIUS_M,
            auto_capture_home: true,
            auto_rtl_on_complete: false,
        }
    }
}

/// Navigation status report
#[derive(Debug, Clone, Copy, Default)]
pub struct NavStatus {
    /// Current navigation state
    pub state: NavState,
    /// Current position (latitude, longitude) if available
    pub position: Option<(f64, f64)>,
    /// Current heading in degrees (0-360) if available
    pub heading: Option<f32>,
    /// Distance to current waypoint in meters
    pub distance_to_waypoint_m: f32,
    /// Bearing to current waypoint in degrees (0-360)
    pub bearing_to_waypoint_deg: f32,
    /// Heading error in degrees (-180 to 180)
    pub heading_error_deg: f32,
    /// Current waypoint index (0-based)
    pub waypoint_index: u8,
    /// Total waypoints in mission
    pub waypoint_count: u8,
    /// Time since last GPS update (ms)
    pub gps_age_ms: u32,
    /// Time since last compass update (ms)
    pub compass_age_ms: u32,
    /// Current commanded speed
    pub commanded_speed: f32,
    /// Current commanded steering
    pub commanded_steering: f32,
    /// Distance to home in meters
    pub home_distance_m: f32,
    /// Whether home position is valid
    pub home_valid: bool,
    /// Home source type
    pub home_source: HomeSource,
}

// ============================================================================
// Navigation Telemetry
// ============================================================================

/// Compact navigation telemetry for streaming (32 bytes)
///
/// All values are scaled integers for efficient transmission.
/// Use to_bytes()/from_bytes() for wire format.
#[derive(Debug, Clone, Copy, Default)]
pub struct NavTelemetry {
    /// Navigation state (NavState as u8)
    pub state: u8,
    /// Vehicle mode (if integrated with vehicle state machine)
    pub vehicle_mode: u8,
    /// Latitude in 1e-7 degrees (fits in i32)
    pub lat_e7: i32,
    /// Longitude in 1e-7 degrees (fits in i32)
    pub lon_e7: i32,
    /// Heading in centidegrees (0-36000)
    pub heading_cdeg: u16,
    /// Distance to current waypoint in meters (u16 max ~65km)
    pub wp_dist_m: u16,
    /// Current waypoint index
    pub wp_index: u8,
    /// Total waypoint count
    pub wp_count: u8,
    /// Distance to home in meters
    pub home_dist_m: u16,
    /// Number of GPS satellites
    pub satellites: u8,
    /// Mission flags (bit 0: auto_rtl, bit 1: loop)
    pub flags: u8,
    /// Timestamp (ms since boot, wraps every ~49 days)
    pub timestamp_ms: u32,
    /// Reserved for future use
    pub _reserved: [u8; 4],
}

impl NavTelemetry {
    /// Size in bytes
    pub const SIZE: usize = 32;

    /// Create telemetry from NavStatus and additional data
    pub fn from_status(
        status: &NavStatus,
        vehicle_mode: u8,
        satellites: u8,
        mission_flags: &MissionFlags,
        timestamp_ms: u32,
    ) -> Self {
        Self {
            state: status.state as u8,
            vehicle_mode,
            lat_e7: status
                .position
                .map(|(lat, _)| (lat * 1e7) as i32)
                .unwrap_or(0),
            lon_e7: status
                .position
                .map(|(_, lon)| (lon * 1e7) as i32)
                .unwrap_or(0),
            heading_cdeg: status
                .heading
                .map(|h| (h * 100.0) as u16)
                .unwrap_or(0),
            wp_dist_m: status.distance_to_waypoint_m as u16,
            wp_index: status.waypoint_index,
            wp_count: status.waypoint_count,
            home_dist_m: status.home_distance_m as u16,
            satellites,
            flags: (mission_flags.auto_rtl as u8) | ((mission_flags.loop_mission as u8) << 1),
            timestamp_ms,
            _reserved: [0; 4],
        }
    }

    /// Serialize to bytes (native endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0] = self.state;
        buf[1] = self.vehicle_mode;
        buf[2..6].copy_from_slice(&self.lat_e7.to_le_bytes());
        buf[6..10].copy_from_slice(&self.lon_e7.to_le_bytes());
        buf[10..12].copy_from_slice(&self.heading_cdeg.to_le_bytes());
        buf[12..14].copy_from_slice(&self.wp_dist_m.to_le_bytes());
        buf[14] = self.wp_index;
        buf[15] = self.wp_count;
        buf[16..18].copy_from_slice(&self.home_dist_m.to_le_bytes());
        buf[18] = self.satellites;
        buf[19] = self.flags;
        buf[20..24].copy_from_slice(&self.timestamp_ms.to_le_bytes());
        buf[24..28].copy_from_slice(&self._reserved);
        buf
    }

    /// Deserialize from bytes
    pub fn from_bytes(buf: &[u8; Self::SIZE]) -> Self {
        Self {
            state: buf[0],
            vehicle_mode: buf[1],
            lat_e7: i32::from_le_bytes([buf[2], buf[3], buf[4], buf[5]]),
            lon_e7: i32::from_le_bytes([buf[6], buf[7], buf[8], buf[9]]),
            heading_cdeg: u16::from_le_bytes([buf[10], buf[11]]),
            wp_dist_m: u16::from_le_bytes([buf[12], buf[13]]),
            wp_index: buf[14],
            wp_count: buf[15],
            home_dist_m: u16::from_le_bytes([buf[16], buf[17]]),
            satellites: buf[18],
            flags: buf[19],
            timestamp_ms: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
            _reserved: [buf[24], buf[25], buf[26], buf[27]],
        }
    }

    /// Get latitude in degrees
    pub fn latitude(&self) -> f64 {
        self.lat_e7 as f64 / 1e7
    }

    /// Get longitude in degrees
    pub fn longitude(&self) -> f64 {
        self.lon_e7 as f64 / 1e7
    }

    /// Get heading in degrees
    pub fn heading(&self) -> f32 {
        self.heading_cdeg as f32 / 100.0
    }
}

/// Commands to NavServer
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavCommand {
    /// Start navigation from current waypoint
    Start,
    /// Pause navigation (stop motors, hold position)
    Pause,
    /// Resume from paused state
    Resume,
    /// Stop and clear mission
    ClearMission,
    /// Add waypoint to end of mission
    AddWaypoint(Waypoint),
    /// Skip to specific waypoint index
    SkipTo(u8),
    /// Go to previous waypoint
    Previous,
    /// Return to home position (RTL)
    ReturnToHome,
    /// Set current GPS position as home
    SetHomeHere,
    /// Set explicit home position
    SetHome { latitude: f64, longitude: f64 },
    /// Cancel RTL and return to Idle
    CancelRtl,
}

// ============================================================================
// NavServer
// ============================================================================

/// Navigation server for autonomous waypoint following
pub struct NavServer {
    // Configuration
    config: NavConfig,

    // State machine
    state: NavState,

    // Mission
    waypoints: [Option<Waypoint>; MAX_WAYPOINTS],
    waypoint_count: u8,
    current_waypoint: u8,
    mission_id: u16,
    mission_flags: MissionFlags,

    // Home position (RTL)
    home: HomePosition,
    first_fix_captured: bool,

    // Sensor data (cached from IPC)
    last_gps: Option<GpsData>,
    last_compass: Option<CompassData>,
    gps_age_ms: u32,
    compass_age_ms: u32,

    // Timing (for home capture timestamp)
    elapsed_total_ms: u32,

    // Output tracking
    last_speed: f32,
    last_steering: f32,
}

impl NavServer {
    /// Create a new navigation server with the given configuration
    pub fn new(config: NavConfig) -> Self {
        Self {
            config,
            state: NavState::Idle,
            waypoints: [None; MAX_WAYPOINTS],
            waypoint_count: 0,
            current_waypoint: 0,
            mission_id: 0,
            mission_flags: MissionFlags::default(),
            home: HomePosition::default(),
            first_fix_captured: false,
            last_gps: None,
            last_compass: None,
            gps_age_ms: u32::MAX,
            compass_age_ms: u32::MAX,
            elapsed_total_ms: 0,
            last_speed: 0.0,
            last_steering: 0.0,
        }
    }

    /// Get current home position
    pub fn home(&self) -> &HomePosition {
        &self.home
    }

    /// Check if home is set and valid
    pub fn has_valid_home(&self) -> bool {
        self.home.valid
    }

    /// Distance from current position to home
    pub fn home_distance(&self) -> Option<f32> {
        if !self.home.valid {
            return None;
        }
        self.last_gps.map(|gps| {
            nav_math::haversine_distance(
                gps.latitude,
                gps.longitude,
                self.home.latitude,
                self.home.longitude,
            )
        })
    }

    /// Check if a mission is loaded
    pub fn has_mission(&self) -> bool {
        self.waypoint_count > 0
    }

    /// Get current mission ID
    pub fn mission_id(&self) -> u16 {
        self.mission_id
    }

    /// Get current mission flags
    pub fn mission_flags(&self) -> &MissionFlags {
        &self.mission_flags
    }

    /// Load a mission from a Mission struct
    ///
    /// Returns false if the mission checksum is invalid.
    pub fn load_mission(&mut self, mission: &Mission) -> bool {
        // Validate checksum
        if !mission.validate() {
            return false;
        }

        // Clear current mission
        self.waypoints = [None; MAX_WAYPOINTS];
        self.waypoint_count = 0;
        self.current_waypoint = 0;
        self.state = NavState::Idle;

        // Copy waypoints
        for i in 0..mission.meta.count as usize {
            if let Some(wp) = mission.waypoints[i] {
                self.waypoints[i] = Some(wp);
                self.waypoint_count += 1;
            }
        }

        // Copy metadata
        self.mission_id = mission.meta.id;
        self.mission_flags = mission.flags;

        // If mission defines home, use it
        if let Some(home) = mission.home {
            self.home = home;
            self.first_fix_captured = true; // Don't overwrite with first fix
        }

        true
    }

    /// Get current navigation state
    pub fn state(&self) -> NavState {
        self.state
    }

    /// Get number of waypoints in mission
    pub fn waypoint_count(&self) -> u8 {
        self.waypoint_count
    }

    /// Get current waypoint index
    pub fn current_waypoint_index(&self) -> u8 {
        self.current_waypoint
    }

    /// Process new GPS data
    ///
    /// Automatically captures first valid fix as home if auto_capture_home is enabled.
    pub fn process_gps(&mut self, gps: GpsData) {
        if gps.is_valid() {
            self.last_gps = Some(gps);
            self.gps_age_ms = 0;

            // Auto-capture first fix as home
            if self.config.auto_capture_home && !self.first_fix_captured {
                self.home = HomePosition::from_gps(&gps, self.elapsed_total_ms, HomeSource::FirstFix);
                self.first_fix_captured = true;
            }
        }
    }

    /// Manually set home at current GPS position
    pub fn set_home_here(&mut self) -> bool {
        if let Some(gps) = self.last_gps {
            if gps.is_valid() {
                self.home = HomePosition::from_gps(&gps, self.elapsed_total_ms, HomeSource::Manual);
                self.first_fix_captured = true; // Prevent auto-capture from overwriting
                return true;
            }
        }
        false
    }

    /// Set explicit home position
    pub fn set_home(&mut self, latitude: f64, longitude: f64) {
        self.home = HomePosition::new(latitude, longitude, 0.0, self.elapsed_total_ms, HomeSource::Manual);
        self.first_fix_captured = true; // Prevent auto-capture from overwriting
    }

    /// Process new compass data
    pub fn process_compass(&mut self, compass: CompassData) {
        if compass.valid {
            self.last_compass = Some(compass);
            self.compass_age_ms = 0;
        }
    }

    /// Process a navigation command
    pub fn process_command(&mut self, cmd: NavCommand) {
        match cmd {
            NavCommand::Start => {
                if self.waypoint_count > 0 {
                    self.state = NavState::Active;
                }
            }
            NavCommand::Pause => {
                if self.state == NavState::Active || self.state == NavState::ReturningHome {
                    self.state = NavState::Paused;
                }
            }
            NavCommand::Resume => {
                if self.state == NavState::Paused {
                    // Return to Active if we have waypoints, otherwise Idle
                    if self.waypoint_count > 0 {
                        self.state = NavState::Active;
                    } else {
                        self.state = NavState::Idle;
                    }
                }
            }
            NavCommand::ClearMission => {
                self.state = NavState::Idle;
                self.waypoints = [None; MAX_WAYPOINTS];
                self.waypoint_count = 0;
                self.current_waypoint = 0;
                self.last_speed = 0.0;
                self.last_steering = 0.0;
            }
            NavCommand::AddWaypoint(wp) => {
                if (self.waypoint_count as usize) < MAX_WAYPOINTS {
                    self.waypoints[self.waypoint_count as usize] = Some(wp);
                    self.waypoint_count += 1;
                }
            }
            NavCommand::SkipTo(idx) => {
                if idx < self.waypoint_count {
                    self.current_waypoint = idx;
                }
            }
            NavCommand::Previous => {
                if self.current_waypoint > 0 {
                    self.current_waypoint -= 1;
                    // Return to Active if was Arrived
                    if self.state == NavState::Arrived {
                        self.state = NavState::Active;
                    }
                }
            }
            NavCommand::ReturnToHome => {
                // Only RTL if we have a valid home
                if self.home.valid {
                    self.state = NavState::ReturningHome;
                } else {
                    // No valid home - enter Hold
                    self.state = NavState::Hold;
                }
            }
            NavCommand::SetHomeHere => {
                self.set_home_here();
            }
            NavCommand::SetHome { latitude, longitude } => {
                self.set_home(latitude, longitude);
            }
            NavCommand::CancelRtl => {
                if self.state == NavState::ReturningHome || self.state == NavState::HomeReached {
                    self.state = NavState::Idle;
                    self.last_speed = 0.0;
                    self.last_steering = 0.0;
                }
            }
        }
    }

    /// Run one navigation update cycle
    ///
    /// Call this at regular intervals (e.g., 10 Hz)
    ///
    /// # Arguments
    /// * `elapsed_ms` - Time since last update in milliseconds
    ///
    /// # Returns
    /// Motor command if navigation is producing output
    pub fn update(&mut self, elapsed_ms: u32) -> Option<MotorCommand> {
        // Update timers
        self.gps_age_ms = self.gps_age_ms.saturating_add(elapsed_ms);
        self.compass_age_ms = self.compass_age_ms.saturating_add(elapsed_ms);
        self.elapsed_total_ms = self.elapsed_total_ms.saturating_add(elapsed_ms);

        // State machine
        match self.state {
            NavState::Idle | NavState::Arrived | NavState::Paused | NavState::HomeReached => {
                self.last_speed = 0.0;
                self.last_steering = 0.0;
                Some(MotorCommand::Coast)
            }
            NavState::Hold => {
                // Check if we can resume navigation
                if self.can_navigate() {
                    self.state = NavState::Active;
                    self.navigate()
                } else {
                    self.last_speed = 0.0;
                    self.last_steering = 0.0;
                    Some(MotorCommand::Coast)
                }
            }
            NavState::Active => {
                // Check for sensor loss
                if !self.can_navigate() {
                    self.state = NavState::Hold;
                    self.last_speed = 0.0;
                    self.last_steering = 0.0;
                    Some(MotorCommand::Coast)
                } else {
                    self.navigate()
                }
            }
            NavState::ReturningHome => {
                // Check for sensor loss
                if !self.can_navigate() {
                    self.state = NavState::Hold;
                    self.last_speed = 0.0;
                    self.last_steering = 0.0;
                    Some(MotorCommand::Coast)
                } else {
                    self.navigate_rtl()
                }
            }
        }
    }

    /// Get current navigation status
    pub fn status(&self) -> NavStatus {
        let mut status = NavStatus {
            state: self.state,
            waypoint_index: self.current_waypoint,
            waypoint_count: self.waypoint_count,
            gps_age_ms: self.gps_age_ms,
            compass_age_ms: self.compass_age_ms,
            commanded_speed: self.last_speed,
            commanded_steering: self.last_steering,
            home_valid: self.home.valid,
            home_source: self.home.source,
            ..Default::default()
        };

        // Fill position if available
        if let Some(gps) = self.last_gps {
            status.position = Some((gps.latitude, gps.longitude));

            // Calculate home distance if we have valid home
            if self.home.valid {
                status.home_distance_m = nav_math::haversine_distance(
                    gps.latitude,
                    gps.longitude,
                    self.home.latitude,
                    self.home.longitude,
                );
            }
        }

        // Fill heading if available
        if let Some(compass) = self.last_compass {
            status.heading = Some(compass.heading_true);
        }

        // Calculate waypoint info if we have data
        if let (Some(gps), Some(waypoint)) = (
            self.last_gps,
            self.waypoints
                .get(self.current_waypoint as usize)
                .and_then(|w| *w),
        ) {
            status.distance_to_waypoint_m = nav_math::haversine_distance(
                gps.latitude,
                gps.longitude,
                waypoint.latitude,
                waypoint.longitude,
            );
            status.bearing_to_waypoint_deg = nav_math::calculate_bearing(
                gps.latitude,
                gps.longitude,
                waypoint.latitude,
                waypoint.longitude,
            );
            if let Some(heading) = status.heading {
                status.heading_error_deg =
                    nav_math::heading_error(heading, status.bearing_to_waypoint_deg);
            }
        }

        status
    }

    /// Generate compact telemetry for streaming
    ///
    /// # Arguments
    /// * `vehicle_mode` - Current vehicle mode (from VehicleStateMachine)
    /// * `timestamp_ms` - Current timestamp in milliseconds
    pub fn telemetry(&self, vehicle_mode: u8, timestamp_ms: u32) -> NavTelemetry {
        let status = self.status();
        let satellites = self.last_gps.map(|g| g.satellites).unwrap_or(0);
        NavTelemetry::from_status(&status, vehicle_mode, satellites, &self.mission_flags, timestamp_ms)
    }

    /// Check if we have valid sensor data for navigation
    fn can_navigate(&self) -> bool {
        // GPS check
        let gps_ok = self.gps_age_ms < self.config.gps_timeout_ms
            && self
                .last_gps
                .map(|g| g.is_valid() && g.hdop <= self.config.max_hdop)
                .unwrap_or(false);

        // Compass check
        let compass_ok = self.compass_age_ms < self.config.compass_timeout_ms
            && self.last_compass.map(|c| c.valid).unwrap_or(false);

        // Compass calibration check
        let compass_cal_ok = !self.config.require_compass_cal
            || self
                .last_compass
                .map(|c| {
                    matches!(
                        c.calibration,
                        crate::compass::CalibrationStatus::Calibrated
                    )
                })
                .unwrap_or(false);

        gps_ok && compass_ok && compass_cal_ok
    }

    /// Core navigation logic
    fn navigate(&mut self) -> Option<MotorCommand> {
        let gps = self.last_gps?;
        let compass = self.last_compass?;
        let waypoint = self.waypoints[self.current_waypoint as usize]?;

        // Calculate navigation values
        let distance = nav_math::haversine_distance(
            gps.latitude,
            gps.longitude,
            waypoint.latitude,
            waypoint.longitude,
        );

        let bearing = nav_math::calculate_bearing(
            gps.latitude,
            gps.longitude,
            waypoint.latitude,
            waypoint.longitude,
        );

        let heading_error = nav_math::heading_error(compass.heading_true, bearing);

        // Check waypoint arrival
        let arrival_radius = if waypoint.radius > 0.0 {
            waypoint.radius
        } else {
            self.config.default_radius_m
        };

        if distance < arrival_radius {
            self.advance_waypoint();
            // Coast for one cycle while transitioning
            self.last_speed = 0.0;
            self.last_steering = 0.0;
            return Some(MotorCommand::Coast);
        }

        // Calculate speed (slow on approach)
        let speed = nav_math::calculate_speed(
            distance,
            self.config.approach_distance_m,
            self.config.max_speed,
            self.config.min_speed,
        );

        // Calculate steering (proportional control with deadband)
        let steering = if heading_error.abs() < self.config.steering_deadband_deg {
            0.0
        } else {
            (heading_error * self.config.steering_kp)
                .clamp(-self.config.max_steering, self.config.max_steering)
        };

        // Convert to motor command
        let (left, right) =
            nav_math::differential_mix(speed, steering, self.config.max_motor_velocity);

        self.last_speed = speed;
        self.last_steering = steering;

        Some(MotorCommand::SetVelocity { left, right })
    }

    /// Advance to next waypoint
    fn advance_waypoint(&mut self) {
        if self.current_waypoint + 1 < self.waypoint_count {
            self.current_waypoint += 1;
        } else {
            // Mission complete - check for loop
            if self.mission_flags.loop_mission {
                // Restart from first waypoint
                self.current_waypoint = 0;
            } else {
                // Check for auto-RTL (mission flag takes precedence over config)
                let auto_rtl = self.mission_flags.auto_rtl || self.config.auto_rtl_on_complete;
                if auto_rtl && self.home.valid {
                    self.state = NavState::ReturningHome;
                } else {
                    self.state = NavState::Arrived;
                }
            }
        }
    }

    /// RTL navigation logic - navigate to home position
    fn navigate_rtl(&mut self) -> Option<MotorCommand> {
        let gps = self.last_gps?;
        let compass = self.last_compass?;

        if !self.home.valid {
            // No valid home - coast
            self.state = NavState::Hold;
            self.last_speed = 0.0;
            self.last_steering = 0.0;
            return Some(MotorCommand::Coast);
        }

        // Calculate navigation to home
        let distance = nav_math::haversine_distance(
            gps.latitude,
            gps.longitude,
            self.home.latitude,
            self.home.longitude,
        );

        let bearing = nav_math::calculate_bearing(
            gps.latitude,
            gps.longitude,
            self.home.latitude,
            self.home.longitude,
        );

        let heading_error = nav_math::heading_error(compass.heading_true, bearing);

        // Check home arrival
        if distance < self.config.home_radius_m {
            self.state = NavState::HomeReached;
            self.last_speed = 0.0;
            self.last_steering = 0.0;
            return Some(MotorCommand::Coast);
        }

        // Calculate speed (slow on approach)
        let speed = nav_math::calculate_speed(
            distance,
            self.config.approach_distance_m,
            self.config.max_speed,
            self.config.min_speed,
        );

        // Calculate steering (proportional control with deadband)
        let steering = if heading_error.abs() < self.config.steering_deadband_deg {
            0.0
        } else {
            (heading_error * self.config.steering_kp)
                .clamp(-self.config.max_steering, self.config.max_steering)
        };

        // Convert to motor command
        let (left, right) =
            nav_math::differential_mix(speed, steering, self.config.max_motor_velocity);

        self.last_speed = speed;
        self.last_steering = steering;

        Some(MotorCommand::SetVelocity { left, right })
    }
}

impl Default for NavServer {
    fn default() -> Self {
        Self::new(NavConfig::default())
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compass::{CalibrationStatus, MagRaw};
    use crate::gps::GpsFix;

    // ========== Math Tests ==========

    #[test]
    fn haversine_zero_distance() {
        let d = nav_math::haversine_distance(47.0, 8.0, 47.0, 8.0);
        assert!(d < 0.01, "Same point should have ~0 distance, got {}", d);
    }

    #[test]
    fn haversine_known_distance() {
        // Zurich (47.3769, 8.5417) to Geneva (46.2044, 6.1432) ~225 km
        let d = nav_math::haversine_distance(47.3769, 8.5417, 46.2044, 6.1432);
        let expected = 225_000.0;
        assert!(
            (d - expected).abs() < 5000.0,
            "Zurich-Geneva ~225km, got {}",
            d
        );
    }

    #[test]
    fn haversine_short_distance() {
        // ~100m north (roughly 0.0009 degrees)
        let d = nav_math::haversine_distance(47.0, 8.0, 47.0009, 8.0);
        assert!(
            (d - 100.0).abs() < 5.0,
            "~100m north, got {} meters",
            d
        );
    }

    #[test]
    fn bearing_north() {
        let b = nav_math::calculate_bearing(0.0, 0.0, 1.0, 0.0);
        assert!(b < 1.0 || b > 359.0, "Due north should be ~0 deg, got {}", b);
    }

    #[test]
    fn bearing_east() {
        let b = nav_math::calculate_bearing(0.0, 0.0, 0.0, 1.0);
        assert!(
            (b - 90.0).abs() < 1.0,
            "Due east should be ~90 deg, got {}",
            b
        );
    }

    #[test]
    fn bearing_south() {
        let b = nav_math::calculate_bearing(0.0, 0.0, -1.0, 0.0);
        assert!(
            (b - 180.0).abs() < 1.0,
            "Due south should be ~180 deg, got {}",
            b
        );
    }

    #[test]
    fn bearing_west() {
        let b = nav_math::calculate_bearing(0.0, 0.0, 0.0, -1.0);
        assert!(
            (b - 270.0).abs() < 1.0,
            "Due west should be ~270 deg, got {}",
            b
        );
    }

    #[test]
    fn heading_error_straight() {
        let e = nav_math::heading_error(0.0, 0.0);
        assert!(e.abs() < 0.01, "Same heading = 0 error, got {}", e);
    }

    #[test]
    fn heading_error_right_turn() {
        let e = nav_math::heading_error(0.0, 90.0);
        assert!(
            (e - 90.0).abs() < 0.01,
            "90 deg right turn, got {}",
            e
        );
    }

    #[test]
    fn heading_error_left_turn() {
        let e = nav_math::heading_error(90.0, 0.0);
        assert!(
            (e - (-90.0)).abs() < 0.01,
            "90 deg left turn, got {}",
            e
        );
    }

    #[test]
    fn heading_error_wraparound_right() {
        // Heading 350, want 10 -> should be +20 (turn right)
        let e = nav_math::heading_error(350.0, 10.0);
        assert!(
            (e - 20.0).abs() < 0.01,
            "350->10 should be +20, got {}",
            e
        );
    }

    #[test]
    fn heading_error_wraparound_left() {
        // Heading 10, want 350 -> should be -20 (turn left)
        let e = nav_math::heading_error(10.0, 350.0);
        assert!(
            (e - (-20.0)).abs() < 0.01,
            "10->350 should be -20, got {}",
            e
        );
    }

    #[test]
    fn speed_at_distance() {
        let s = nav_math::calculate_speed(100.0, 10.0, 0.5, 0.1);
        assert!((s - 0.5).abs() < 0.01, "Far away = max speed");
    }

    #[test]
    fn speed_at_approach() {
        let s = nav_math::calculate_speed(5.0, 10.0, 0.5, 0.1);
        // At 5m with 10m approach, should be 50% of range
        let expected = 0.1 + (0.5 - 0.1) * 0.5; // 0.3
        assert!(
            (s - expected).abs() < 0.01,
            "Halfway = mid speed, got {}",
            s
        );
    }

    #[test]
    fn speed_at_arrival() {
        let s = nav_math::calculate_speed(0.0, 10.0, 0.5, 0.1);
        assert!((s - 0.1).abs() < 0.01, "At waypoint = min speed");
    }

    #[test]
    fn differential_mix_forward() {
        let (l, r) = nav_math::differential_mix(0.5, 0.0, 1000);
        assert_eq!(l, 500);
        assert_eq!(r, 500);
    }

    #[test]
    fn differential_mix_turn_right() {
        let (l, r) = nav_math::differential_mix(0.5, 0.2, 1000);
        assert!(l > r, "Right turn: left > right");
        assert_eq!(l, 700);
        assert_eq!(r, 300);
    }

    #[test]
    fn differential_mix_turn_left() {
        let (l, r) = nav_math::differential_mix(0.5, -0.2, 1000);
        assert!(r > l, "Left turn: right > left");
        assert_eq!(l, 300);
        assert_eq!(r, 700);
    }

    // ========== State Machine Tests ==========

    #[test]
    fn nav_starts_idle() {
        let nav = NavServer::new(NavConfig::default());
        assert_eq!(nav.state(), NavState::Idle);
    }

    #[test]
    fn nav_add_waypoint() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        assert_eq!(nav.waypoint_count(), 1);
    }

    #[test]
    fn nav_add_multiple_waypoints() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.1)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.2, 8.2)));
        assert_eq!(nav.waypoint_count(), 3);
    }

    #[test]
    fn nav_start_without_waypoints_stays_idle() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::Start);
        assert_eq!(nav.state(), NavState::Idle);
    }

    #[test]
    fn nav_start_with_waypoints_goes_active() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::Start);
        assert_eq!(nav.state(), NavState::Active);
    }

    #[test]
    fn nav_enters_hold_without_sensors() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::Start);

        // Update without any sensor data
        nav.update(100);

        assert_eq!(nav.state(), NavState::Hold);
    }

    #[test]
    fn nav_pause_and_resume() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::Start);

        nav.process_command(NavCommand::Pause);
        assert_eq!(nav.state(), NavState::Paused);

        nav.process_command(NavCommand::Resume);
        assert_eq!(nav.state(), NavState::Active);
    }

    #[test]
    fn nav_clear_mission() {
        let mut nav = NavServer::new(NavConfig::default());
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.1)));
        nav.process_command(NavCommand::Start);

        nav.process_command(NavCommand::ClearMission);

        assert_eq!(nav.state(), NavState::Idle);
        assert_eq!(nav.waypoint_count(), 0);
    }

    // ========== Navigation Tests ==========

    fn make_valid_gps(lat: f64, lon: f64) -> GpsData {
        GpsData {
            latitude: lat,
            longitude: lon,
            altitude: 100.0,
            speed: 1.0,
            course: 0.0,
            fix: GpsFix::Fix3D,
            hdop: 1.0,
            satellites: 10,
            timestamp_ms: 0,
            valid: true,
        }
    }

    fn make_valid_compass(heading: f32) -> CompassData {
        CompassData {
            heading,
            heading_true: heading,
            raw: MagRaw::default(),
            calibration: CalibrationStatus::Calibrated,
            valid: true,
        }
    }

    #[test]
    fn nav_proceeds_with_valid_sensors() {
        let mut nav = NavServer::new(NavConfig::default());

        // Add waypoint 100m north
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.001, 8.0)));
        nav.process_command(NavCommand::Start);

        // Feed valid sensor data
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        let cmd = nav.update(100);

        assert!(cmd.is_some());
        assert_eq!(nav.state(), NavState::Active);
    }

    #[test]
    fn nav_produces_velocity_command() {
        let mut nav = NavServer::new(NavConfig::default());

        // Waypoint due north
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.001, 8.0)));
        nav.process_command(NavCommand::Start);

        // Position south, heading north
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        let cmd = nav.update(100);

        match cmd {
            Some(MotorCommand::SetVelocity { left, right }) => {
                // Heading is correct (north), so left ~= right
                assert!(left > 0);
                assert!(right > 0);
                assert!((left - right).abs() < 100); // Small steering
            }
            _ => panic!("Expected SetVelocity command"),
        }
    }

    #[test]
    fn nav_steers_to_waypoint() {
        let mut nav = NavServer::new(NavConfig::default());

        // Waypoint due east
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.001)));
        nav.process_command(NavCommand::Start);

        // Position, heading north (need to turn right)
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0)); // Heading north

        let cmd = nav.update(100);

        match cmd {
            Some(MotorCommand::SetVelocity { left, right }) => {
                // Need to turn right: left > right
                assert!(left > right, "Should turn right: left={} right={}", left, right);
            }
            _ => panic!("Expected SetVelocity command"),
        }
    }

    #[test]
    fn nav_arrives_at_waypoint() {
        let mut config = NavConfig::default();
        config.default_radius_m = 10.0; // Large radius for test
        let mut nav = NavServer::new(config);

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::Start);

        // Position at waypoint
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        nav.update(100);

        // Single waypoint mission -> Arrived
        assert_eq!(nav.state(), NavState::Arrived);
    }

    #[test]
    fn nav_advances_to_next_waypoint() {
        let mut config = NavConfig::default();
        config.default_radius_m = 10.0;
        let mut nav = NavServer::new(config);

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.0)));
        nav.process_command(NavCommand::Start);

        // Arrive at first waypoint
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));
        nav.update(100);

        // Should advance, still Active
        assert_eq!(nav.state(), NavState::Active);
        assert_eq!(nav.current_waypoint_index(), 1);
    }

    #[test]
    fn nav_status_reports_distance() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.001, 8.0)));
        nav.process_command(NavCommand::Start);

        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        let status = nav.status();

        // ~100m north
        assert!(
            (status.distance_to_waypoint_m - 111.0).abs() < 20.0,
            "Distance ~111m, got {}",
            status.distance_to_waypoint_m
        );
    }

    #[test]
    fn nav_skip_to_waypoint() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.2, 8.0)));

        nav.process_command(NavCommand::SkipTo(2));
        assert_eq!(nav.current_waypoint_index(), 2);
    }

    #[test]
    fn nav_previous_waypoint() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.0)));
        nav.process_command(NavCommand::SkipTo(1));

        nav.process_command(NavCommand::Previous);
        assert_eq!(nav.current_waypoint_index(), 0);
    }

    // ========== RTL Tests ==========

    #[test]
    fn rtl_first_fix_captures_home() {
        let mut nav = NavServer::new(NavConfig::default());

        // Initially no home
        assert!(!nav.has_valid_home());

        // First GPS fix should capture home
        nav.process_gps(make_valid_gps(47.0, 8.0));

        assert!(nav.has_valid_home());
        assert_eq!(nav.home().source, HomeSource::FirstFix);
        assert!((nav.home().latitude - 47.0).abs() < 0.0001);
        assert!((nav.home().longitude - 8.0).abs() < 0.0001);
    }

    #[test]
    fn rtl_first_fix_only_captured_once() {
        let mut nav = NavServer::new(NavConfig::default());

        // First fix
        nav.process_gps(make_valid_gps(47.0, 8.0));
        assert!((nav.home().latitude - 47.0).abs() < 0.0001);

        // Second fix should NOT overwrite
        nav.process_gps(make_valid_gps(48.0, 9.0));
        assert!((nav.home().latitude - 47.0).abs() < 0.0001);
    }

    #[test]
    fn rtl_set_home_here() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_gps(make_valid_gps(47.0, 8.0));

        // Move to new location
        nav.process_gps(make_valid_gps(48.0, 9.0));

        // Set new home
        nav.process_command(NavCommand::SetHomeHere);

        assert_eq!(nav.home().source, HomeSource::Manual);
        assert!((nav.home().latitude - 48.0).abs() < 0.0001);
    }

    #[test]
    fn rtl_set_explicit_home() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_command(NavCommand::SetHome {
            latitude: 50.0,
            longitude: 10.0,
        });

        assert!(nav.has_valid_home());
        assert_eq!(nav.home().source, HomeSource::Manual);
        assert!((nav.home().latitude - 50.0).abs() < 0.0001);
    }

    #[test]
    fn rtl_return_to_home_enters_returning_state() {
        let mut nav = NavServer::new(NavConfig::default());

        // Set home
        nav.process_gps(make_valid_gps(47.0, 8.0));

        // RTL command
        nav.process_command(NavCommand::ReturnToHome);

        assert_eq!(nav.state(), NavState::ReturningHome);
    }

    #[test]
    fn rtl_no_home_enters_hold() {
        let mut config = NavConfig::default();
        config.auto_capture_home = false;
        let mut nav = NavServer::new(config);

        // No home set
        nav.process_command(NavCommand::ReturnToHome);

        assert_eq!(nav.state(), NavState::Hold);
    }

    #[test]
    fn rtl_arrives_at_home() {
        let mut config = NavConfig::default();
        config.home_radius_m = 10.0;
        let mut nav = NavServer::new(config);

        // Set home at origin
        nav.process_command(NavCommand::SetHome {
            latitude: 47.0,
            longitude: 8.0,
        });

        // Start RTL
        nav.process_command(NavCommand::ReturnToHome);

        // Position at home
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        nav.update(100);

        assert_eq!(nav.state(), NavState::HomeReached);
    }

    #[test]
    fn rtl_navigates_toward_home() {
        let mut nav = NavServer::new(NavConfig::default());

        // Home at origin
        nav.process_command(NavCommand::SetHome {
            latitude: 47.0,
            longitude: 8.0,
        });

        // Start RTL
        nav.process_command(NavCommand::ReturnToHome);

        // Position 100m south, heading north
        nav.process_gps(make_valid_gps(46.999, 8.0));
        nav.process_compass(make_valid_compass(0.0));

        let cmd = nav.update(100);

        match cmd {
            Some(MotorCommand::SetVelocity { left, right }) => {
                // Should be driving forward
                assert!(left > 0);
                assert!(right > 0);
            }
            _ => panic!("Expected SetVelocity for RTL"),
        }
    }

    #[test]
    fn rtl_cancel_returns_to_idle() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_command(NavCommand::ReturnToHome);
        assert_eq!(nav.state(), NavState::ReturningHome);

        nav.process_command(NavCommand::CancelRtl);
        assert_eq!(nav.state(), NavState::Idle);
    }

    #[test]
    fn rtl_home_distance_reported() {
        let mut nav = NavServer::new(NavConfig::default());

        // Home at origin
        nav.process_command(NavCommand::SetHome {
            latitude: 47.0,
            longitude: 8.0,
        });

        // Position ~111m north (0.001 deg lat)
        nav.process_gps(make_valid_gps(47.001, 8.0));

        let dist = nav.home_distance();
        assert!(dist.is_some());
        assert!(
            (dist.unwrap() - 111.0).abs() < 20.0,
            "Expected ~111m, got {}",
            dist.unwrap()
        );
    }

    #[test]
    fn rtl_status_includes_home_info() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_gps(make_valid_gps(47.0, 8.0));

        let status = nav.status();

        assert!(status.home_valid);
        assert_eq!(status.home_source, HomeSource::FirstFix);
    }

    #[test]
    fn rtl_auto_rtl_on_mission_complete() {
        let mut config = NavConfig::default();
        config.default_radius_m = 10.0;
        config.auto_rtl_on_complete = true;
        let mut nav = NavServer::new(config);

        // Set home
        nav.process_command(NavCommand::SetHome {
            latitude: 46.0,
            longitude: 7.0,
        });

        // Single waypoint mission
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::Start);

        // Arrive at waypoint
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));
        nav.update(100);

        // Should auto-RTL instead of Arrived
        assert_eq!(nav.state(), NavState::ReturningHome);
    }

    // ========== Mission Management Tests ==========

    #[test]
    fn mission_create_empty() {
        let mission = Mission::new(42);
        assert_eq!(mission.meta.id, 42);
        assert_eq!(mission.meta.count, 0);
    }

    #[test]
    fn mission_add_waypoints() {
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.add_waypoint(Waypoint::new(47.1, 8.1));
        assert_eq!(mission.waypoint_count(), 2);
    }

    #[test]
    fn mission_checksum_calculation() {
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.finalize();

        assert_ne!(mission.meta.checksum, 0);
    }

    #[test]
    fn mission_checksum_validation() {
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.finalize();

        assert!(mission.validate());
    }

    #[test]
    fn mission_invalid_checksum_rejected() {
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.meta.checksum = 0xDEAD; // Wrong checksum

        assert!(!mission.validate());
    }

    #[test]
    fn mission_load_valid() {
        let mut nav = NavServer::new(NavConfig::default());
        let mut mission = Mission::new(42);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.add_waypoint(Waypoint::new(47.1, 8.1));
        mission.finalize();

        let result = nav.load_mission(&mission);

        assert!(result);
        assert_eq!(nav.waypoint_count(), 2);
        assert_eq!(nav.mission_id(), 42);
    }

    #[test]
    fn mission_load_invalid_rejected() {
        let mut nav = NavServer::new(NavConfig::default());
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.meta.checksum = 0xDEAD;

        let result = nav.load_mission(&mission);

        assert!(!result);
        assert_eq!(nav.waypoint_count(), 0);
    }

    #[test]
    fn mission_with_home_sets_home() {
        let mut nav = NavServer::new(NavConfig::default());
        let mut mission = Mission::new(1);
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.home = Some(HomePosition::new(50.0, 10.0, 0.0, 0, HomeSource::Mission));
        mission.finalize();

        nav.load_mission(&mission);

        assert!(nav.has_valid_home());
        assert_eq!(nav.home().source, HomeSource::Mission);
        assert!((nav.home().latitude - 50.0).abs() < 0.0001);
    }

    #[test]
    fn mission_auto_rtl_flag() {
        let mut config = NavConfig::default();
        config.default_radius_m = 10.0;
        let mut nav = NavServer::new(config);

        // Set home
        nav.process_command(NavCommand::SetHome {
            latitude: 46.0,
            longitude: 7.0,
        });

        // Create mission with auto_rtl flag
        let mut mission = Mission::new(1);
        mission.flags.auto_rtl = true;
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.finalize();

        nav.load_mission(&mission);
        nav.process_command(NavCommand::Start);

        // Arrive at waypoint
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));
        nav.update(100);

        // Should auto-RTL due to mission flag
        assert_eq!(nav.state(), NavState::ReturningHome);
    }

    #[test]
    fn mission_loop_flag() {
        let mut config = NavConfig::default();
        config.default_radius_m = 10.0;
        let mut nav = NavServer::new(config);

        // Create looping mission
        let mut mission = Mission::new(1);
        mission.flags.loop_mission = true;
        mission.add_waypoint(Waypoint::new(47.0, 8.0));
        mission.add_waypoint(Waypoint::new(47.1, 8.1));
        mission.finalize();

        nav.load_mission(&mission);
        nav.process_command(NavCommand::Start);

        // Arrive at both waypoints
        nav.process_gps(make_valid_gps(47.0, 8.0));
        nav.process_compass(make_valid_compass(0.0));
        nav.update(100);
        assert_eq!(nav.current_waypoint_index(), 1);

        nav.process_gps(make_valid_gps(47.1, 8.1));
        nav.update(100);

        // Should loop back to first waypoint
        assert_eq!(nav.state(), NavState::Active);
        assert_eq!(nav.current_waypoint_index(), 0);
    }

    #[test]
    fn mission_has_mission_check() {
        let mut nav = NavServer::new(NavConfig::default());
        assert!(!nav.has_mission());

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        assert!(nav.has_mission());

        nav.process_command(NavCommand::ClearMission);
        assert!(!nav.has_mission());
    }

    // ========== NavTelemetry Tests ==========

    #[test]
    fn telemetry_size_is_32_bytes() {
        assert_eq!(NavTelemetry::SIZE, 32);
    }

    #[test]
    fn telemetry_serialization_roundtrip() {
        let telem = NavTelemetry {
            state: NavState::Active as u8,
            vehicle_mode: 3,
            lat_e7: 470000000, // 47.0 degrees
            lon_e7: 80000000,  // 8.0 degrees
            heading_cdeg: 9000, // 90.0 degrees
            wp_dist_m: 150,
            wp_index: 2,
            wp_count: 5,
            home_dist_m: 500,
            satellites: 10,
            flags: 0b11, // auto_rtl and loop
            timestamp_ms: 123456,
            _reserved: [0; 4],
        };

        let bytes = telem.to_bytes();
        let restored = NavTelemetry::from_bytes(&bytes);

        assert_eq!(restored.state, NavState::Active as u8);
        assert_eq!(restored.vehicle_mode, 3);
        assert_eq!(restored.lat_e7, 470000000);
        assert_eq!(restored.lon_e7, 80000000);
        assert_eq!(restored.heading_cdeg, 9000);
        assert_eq!(restored.wp_dist_m, 150);
        assert_eq!(restored.wp_index, 2);
        assert_eq!(restored.wp_count, 5);
        assert_eq!(restored.home_dist_m, 500);
        assert_eq!(restored.satellites, 10);
        assert_eq!(restored.flags, 0b11);
        assert_eq!(restored.timestamp_ms, 123456);
    }

    #[test]
    fn telemetry_coordinate_conversion() {
        let telem = NavTelemetry {
            lat_e7: 471234567, // 47.1234567 degrees
            lon_e7: 81234567,  // 8.1234567 degrees
            heading_cdeg: 12345, // 123.45 degrees
            ..Default::default()
        };

        assert!((telem.latitude() - 47.1234567).abs() < 1e-7);
        assert!((telem.longitude() - 8.1234567).abs() < 1e-7);
        assert!((telem.heading() - 123.45).abs() < 0.01);
    }

    #[test]
    fn telemetry_from_nav_server() {
        let mut nav = NavServer::new(NavConfig::default());

        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.0, 8.0)));
        nav.process_command(NavCommand::AddWaypoint(Waypoint::new(47.1, 8.1)));
        nav.process_command(NavCommand::Start);

        nav.process_gps(make_valid_gps(46.9, 7.9));
        nav.process_compass(make_valid_compass(45.0));
        nav.update(100);

        let telem = nav.telemetry(2, 1000);

        assert_eq!(telem.state, NavState::Active as u8);
        assert_eq!(telem.vehicle_mode, 2);
        assert_eq!(telem.wp_count, 2);
        assert_eq!(telem.satellites, 10); // from make_valid_gps
        assert_eq!(telem.timestamp_ms, 1000);
    }

    #[test]
    fn telemetry_flags_encode_mission_flags() {
        let flags = MissionFlags {
            auto_rtl: true,
            loop_mission: true,
        };

        let status = NavStatus::default();
        let telem = NavTelemetry::from_status(&status, 0, 0, &flags, 0);

        assert_eq!(telem.flags & 0b01, 1); // auto_rtl
        assert_eq!((telem.flags >> 1) & 0b01, 1); // loop_mission
    }
}
