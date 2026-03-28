//! RC Receiver server
//!
//! Decodes RC receiver protocols and provides normalized channel data.
//!
//! Supported protocols:
//! - SBUS (FrSky, Futaba) - 100kbaud inverted UART
//! - CRSF/ELRS (ExpressLRS, TBS Crossfire) - 420kbaud UART
//! - IBUS (FlySky) - 115200 baud UART
//! - PPM (legacy) - GPIO pulse timing
//!
//! Features:
//! - Normalized channel output [-1.0, 1.0]
//! - Failsafe detection and handling
//! - Link quality / RSSI reporting (where available)
//! - Channel mapping for different vehicle types

/// Maximum number of RC channels
pub const MAX_CHANNELS: usize = 16;

/// SBUS frame size
pub const SBUS_FRAME_SIZE: usize = 25;

/// CRSF max frame size
pub const CRSF_MAX_FRAME_SIZE: usize = 64;

/// RC channel data (normalized)
#[derive(Debug, Clone, Copy, Default)]
pub struct RcChannels {
    /// Channel values [-1.0, 1.0] (center = 0.0)
    pub channels: [f32; MAX_CHANNELS],
    /// Number of valid channels
    pub num_channels: u8,
    /// RSSI (0-100, if available)
    pub rssi: Option<u8>,
    /// Link quality (0-100, if available)
    pub link_quality: Option<u8>,
    /// Failsafe active
    pub failsafe: bool,
    /// Frame lost (current frame)
    pub frame_lost: bool,
}

impl RcChannels {
    /// Get channel value, clamped to [-1.0, 1.0]
    pub fn get(&self, index: usize) -> f32 {
        if index < MAX_CHANNELS {
            self.channels[index].clamp(-1.0, 1.0)
        } else {
            0.0
        }
    }

    /// Check if channels are valid (not failsafe)
    pub fn is_valid(&self) -> bool {
        !self.failsafe && !self.frame_lost
    }
}

/// Failsafe behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FailsafeMode {
    /// Hold last good values
    #[default]
    Hold,
    /// Return to neutral (0.0 on all channels)
    Neutral,
    /// Custom values per channel
    Custom,
}

/// Failsafe configuration
#[derive(Debug, Clone, Copy)]
pub struct FailsafeConfig {
    /// Failsafe behavior mode
    pub mode: FailsafeMode,
    /// Timeout before failsafe triggers (ms)
    pub timeout_ms: u32,
    /// Custom failsafe values (if mode == Custom)
    pub custom_values: [f32; MAX_CHANNELS],
}

impl Default for FailsafeConfig {
    fn default() -> Self {
        Self {
            mode: FailsafeMode::Hold,
            timeout_ms: 500,
            custom_values: [0.0; MAX_CHANNELS],
        }
    }
}

/// SBUS decoder state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SbusState {
    /// Waiting for start byte
    WaitStart,
    /// Receiving data bytes
    Receiving,
}

/// SBUS protocol decoder
///
/// SBUS frame format:
/// - Byte 0: Start (0x0F)
/// - Bytes 1-22: Channel data (11 bits * 16 channels)
/// - Byte 23: Flags (failsafe, frame lost, ch17, ch18)
/// - Byte 24: End (0x00)
#[derive(Debug)]
pub struct SbusDecoder {
    buffer: [u8; SBUS_FRAME_SIZE],
    index: usize,
    state: SbusState,
    last_channels: RcChannels,
}

impl SbusDecoder {
    /// Create a new SBUS decoder
    pub fn new() -> Self {
        Self {
            buffer: [0; SBUS_FRAME_SIZE],
            index: 0,
            state: SbusState::WaitStart,
            last_channels: RcChannels::default(),
        }
    }

    /// Feed a byte to the decoder, returns Some(channels) on complete frame
    pub fn decode(&mut self, byte: u8) -> Option<RcChannels> {
        match self.state {
            SbusState::WaitStart => {
                if byte == 0x0F {
                    self.buffer[0] = byte;
                    self.index = 1;
                    self.state = SbusState::Receiving;
                }
                None
            }
            SbusState::Receiving => {
                self.buffer[self.index] = byte;
                self.index += 1;

                if self.index >= SBUS_FRAME_SIZE {
                    self.state = SbusState::WaitStart;
                    self.index = 0;

                    // Validate end byte
                    if self.buffer[24] == 0x00 {
                        let channels = self.parse_frame();
                        self.last_channels = channels;
                        Some(channels)
                    } else {
                        // Invalid frame, try to resync
                        None
                    }
                } else {
                    None
                }
            }
        }
    }

    /// Parse SBUS frame buffer into channels
    fn parse_frame(&self) -> RcChannels {
        let mut channels = RcChannels::default();
        channels.num_channels = 16;

        // Extract 11-bit channels from packed data
        // Channels are packed LSB first across bytes 1-22
        let data = &self.buffer[1..23];

        // Channel 1-16 extraction (11 bits each)
        let raw_channels: [u16; 16] = [
            ((data[0] as u16) | ((data[1] as u16) << 8)) & 0x07FF,
            ((data[1] as u16 >> 3) | ((data[2] as u16) << 5)) & 0x07FF,
            ((data[2] as u16 >> 6) | ((data[3] as u16) << 2) | ((data[4] as u16) << 10)) & 0x07FF,
            ((data[4] as u16 >> 1) | ((data[5] as u16) << 7)) & 0x07FF,
            ((data[5] as u16 >> 4) | ((data[6] as u16) << 4)) & 0x07FF,
            ((data[6] as u16 >> 7) | ((data[7] as u16) << 1) | ((data[8] as u16) << 9)) & 0x07FF,
            ((data[8] as u16 >> 2) | ((data[9] as u16) << 6)) & 0x07FF,
            ((data[9] as u16 >> 5) | ((data[10] as u16) << 3)) & 0x07FF,
            ((data[11] as u16) | ((data[12] as u16) << 8)) & 0x07FF,
            ((data[12] as u16 >> 3) | ((data[13] as u16) << 5)) & 0x07FF,
            ((data[13] as u16 >> 6) | ((data[14] as u16) << 2) | ((data[15] as u16) << 10)) & 0x07FF,
            ((data[15] as u16 >> 1) | ((data[16] as u16) << 7)) & 0x07FF,
            ((data[16] as u16 >> 4) | ((data[17] as u16) << 4)) & 0x07FF,
            ((data[17] as u16 >> 7) | ((data[18] as u16) << 1) | ((data[19] as u16) << 9)) & 0x07FF,
            ((data[19] as u16 >> 2) | ((data[20] as u16) << 6)) & 0x07FF,
            ((data[20] as u16 >> 5) | ((data[21] as u16) << 3)) & 0x07FF,
        ];

        // Convert to normalized [-1.0, 1.0]
        // SBUS range: 172-1811 (center 992)
        for (i, &raw) in raw_channels.iter().enumerate() {
            channels.channels[i] = sbus_to_normalized(raw);
        }

        // Parse flags byte
        let flags = self.buffer[23];
        channels.frame_lost = (flags & 0x04) != 0;
        channels.failsafe = (flags & 0x08) != 0;

        channels
    }

    /// Get last valid channels
    pub fn last_channels(&self) -> &RcChannels {
        &self.last_channels
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.state = SbusState::WaitStart;
        self.index = 0;
    }
}

impl Default for SbusDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert SBUS raw value (172-1811) to normalized (-1.0 to 1.0)
fn sbus_to_normalized(raw: u16) -> f32 {
    // SBUS range: 172 (min) - 992 (center) - 1811 (max)
    const SBUS_MIN: f32 = 172.0;
    const SBUS_MAX: f32 = 1811.0;
    const SBUS_CENTER: f32 = 992.0;
    const SBUS_RANGE: f32 = (SBUS_MAX - SBUS_MIN) / 2.0;

    let normalized = (raw as f32 - SBUS_CENTER) / SBUS_RANGE;
    normalized.clamp(-1.0, 1.0)
}

/// Convert normalized value (-1.0 to 1.0) to SBUS raw (172-1811)
fn normalized_to_sbus(normalized: f32) -> u16 {
    const SBUS_MIN: f32 = 172.0;
    const SBUS_MAX: f32 = 1811.0;
    const SBUS_CENTER: f32 = 992.0;
    const SBUS_RANGE: f32 = (SBUS_MAX - SBUS_MIN) / 2.0;

    let raw = (normalized * SBUS_RANGE + SBUS_CENTER) as u16;
    raw.clamp(172, 1811)
}

/// CRSF frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CrsfFrameType {
    /// RC channels (packed)
    RcChannels = 0x16,
    /// Link statistics
    LinkStatistics = 0x14,
    /// Battery sensor
    Battery = 0x08,
    /// GPS
    Gps = 0x02,
    /// Attitude
    Attitude = 0x1E,
    /// Flight mode
    FlightMode = 0x21,
    /// Unknown type
    Unknown = 0xFF,
}

impl From<u8> for CrsfFrameType {
    fn from(value: u8) -> Self {
        match value {
            0x16 => CrsfFrameType::RcChannels,
            0x14 => CrsfFrameType::LinkStatistics,
            0x08 => CrsfFrameType::Battery,
            0x02 => CrsfFrameType::Gps,
            0x1E => CrsfFrameType::Attitude,
            0x21 => CrsfFrameType::FlightMode,
            _ => CrsfFrameType::Unknown,
        }
    }
}

/// CRSF decoder state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CrsfState {
    /// Waiting for sync/address byte
    WaitSync,
    /// Got sync, waiting for length
    WaitLength,
    /// Receiving payload
    Receiving,
}

/// CRSF/ELRS protocol decoder
///
/// CRSF frame format:
/// - Byte 0: Address (0xC8 = flight controller)
/// - Byte 1: Frame length (payload + type + CRC)
/// - Byte 2: Frame type
/// - Bytes 3-n: Payload
/// - Byte n+1: CRC8
#[derive(Debug)]
pub struct CrsfDecoder {
    buffer: [u8; CRSF_MAX_FRAME_SIZE],
    index: usize,
    frame_len: usize,
    state: CrsfState,
    last_channels: RcChannels,
    rssi: u8,
    link_quality: u8,
}

impl CrsfDecoder {
    /// CRSF address for flight controller
    pub const ADDR_FLIGHT_CONTROLLER: u8 = 0xC8;

    /// Create a new CRSF decoder
    pub fn new() -> Self {
        Self {
            buffer: [0; CRSF_MAX_FRAME_SIZE],
            index: 0,
            frame_len: 0,
            state: CrsfState::WaitSync,
            last_channels: RcChannels::default(),
            rssi: 0,
            link_quality: 100,
        }
    }

    /// Feed a byte to the decoder, returns Some(channels) on complete RC frame
    pub fn decode(&mut self, byte: u8) -> Option<RcChannels> {
        match self.state {
            CrsfState::WaitSync => {
                if byte == Self::ADDR_FLIGHT_CONTROLLER {
                    self.buffer[0] = byte;
                    self.index = 1;
                    self.state = CrsfState::WaitLength;
                }
                None
            }
            CrsfState::WaitLength => {
                if byte >= 2 && byte <= CRSF_MAX_FRAME_SIZE as u8 - 2 {
                    self.buffer[1] = byte;
                    self.frame_len = byte as usize + 2; // length includes type + payload + crc
                    self.index = 2;
                    self.state = CrsfState::Receiving;
                } else {
                    // Invalid length, reset
                    self.state = CrsfState::WaitSync;
                }
                None
            }
            CrsfState::Receiving => {
                self.buffer[self.index] = byte;
                self.index += 1;

                if self.index >= self.frame_len {
                    self.state = CrsfState::WaitSync;

                    // Validate CRC
                    if self.validate_crc() {
                        self.parse_frame()
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
        }
    }

    /// Validate CRC8 of received frame
    fn validate_crc(&self) -> bool {
        let crc_index = self.frame_len - 1;
        let expected_crc = self.buffer[crc_index];
        let calculated_crc = crsf_crc8(&self.buffer[2..crc_index]);
        expected_crc == calculated_crc
    }

    /// Parse CRSF frame and return channels if RC frame
    fn parse_frame(&mut self) -> Option<RcChannels> {
        let frame_type = CrsfFrameType::from(self.buffer[2]);

        match frame_type {
            CrsfFrameType::RcChannels => {
                let channels = self.parse_rc_channels();
                self.last_channels = channels;
                Some(channels)
            }
            CrsfFrameType::LinkStatistics => {
                self.parse_link_stats();
                None
            }
            _ => None,
        }
    }

    /// Parse RC channels frame (11 bits per channel, 16 channels)
    fn parse_rc_channels(&self) -> RcChannels {
        let mut channels = RcChannels::default();
        channels.num_channels = 16;
        channels.rssi = Some(self.rssi);
        channels.link_quality = Some(self.link_quality);

        let data = &self.buffer[3..];

        // CRSF uses same 11-bit packing as SBUS but different range
        // CRSF range: 172-1811 (same as SBUS for compatibility)
        let raw_channels: [u16; 16] = [
            ((data[0] as u16) | ((data[1] as u16) << 8)) & 0x07FF,
            ((data[1] as u16 >> 3) | ((data[2] as u16) << 5)) & 0x07FF,
            ((data[2] as u16 >> 6) | ((data[3] as u16) << 2) | ((data[4] as u16) << 10)) & 0x07FF,
            ((data[4] as u16 >> 1) | ((data[5] as u16) << 7)) & 0x07FF,
            ((data[5] as u16 >> 4) | ((data[6] as u16) << 4)) & 0x07FF,
            ((data[6] as u16 >> 7) | ((data[7] as u16) << 1) | ((data[8] as u16) << 9)) & 0x07FF,
            ((data[8] as u16 >> 2) | ((data[9] as u16) << 6)) & 0x07FF,
            ((data[9] as u16 >> 5) | ((data[10] as u16) << 3)) & 0x07FF,
            ((data[11] as u16) | ((data[12] as u16) << 8)) & 0x07FF,
            ((data[12] as u16 >> 3) | ((data[13] as u16) << 5)) & 0x07FF,
            ((data[13] as u16 >> 6) | ((data[14] as u16) << 2) | ((data[15] as u16) << 10)) & 0x07FF,
            ((data[15] as u16 >> 1) | ((data[16] as u16) << 7)) & 0x07FF,
            ((data[16] as u16 >> 4) | ((data[17] as u16) << 4)) & 0x07FF,
            ((data[17] as u16 >> 7) | ((data[18] as u16) << 1) | ((data[19] as u16) << 9)) & 0x07FF,
            ((data[19] as u16 >> 2) | ((data[20] as u16) << 6)) & 0x07FF,
            ((data[20] as u16 >> 5) | ((data[21] as u16) << 3)) & 0x07FF,
        ];

        for (i, &raw) in raw_channels.iter().enumerate() {
            channels.channels[i] = sbus_to_normalized(raw); // Same range as SBUS
        }

        channels
    }

    /// Parse link statistics frame
    fn parse_link_stats(&mut self) {
        // Link stats frame payload:
        // [0] Uplink RSSI Ant1
        // [1] Uplink RSSI Ant2
        // [2] Uplink Link Quality
        // [3] Uplink SNR
        // [4] Active antenna
        // [5] RF Mode
        // [6] Uplink TX Power
        // [7] Downlink RSSI
        // [8] Downlink Link Quality
        // [9] Downlink SNR
        if self.frame_len >= 13 {
            let rssi_raw = self.buffer[3];
            self.rssi = if rssi_raw > 0 { 100u8.saturating_sub(rssi_raw) } else { 0 };
            self.link_quality = self.buffer[5];
        }
    }

    /// Get last valid channels
    pub fn last_channels(&self) -> &RcChannels {
        &self.last_channels
    }

    /// Get current RSSI (0-100)
    pub fn rssi(&self) -> u8 {
        self.rssi
    }

    /// Get current link quality (0-100)
    pub fn link_quality(&self) -> u8 {
        self.link_quality
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.state = CrsfState::WaitSync;
        self.index = 0;
    }
}

impl Default for CrsfDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC8 DVB-S2 lookup table for CRSF
const CRSF_CRC8_TABLE: [u8; 256] = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
];

/// Calculate CRC8 DVB-S2 for CRSF
fn crsf_crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc = CRSF_CRC8_TABLE[(crc ^ byte) as usize];
    }
    crc
}

/// Channel mapping configuration
#[derive(Debug, Clone, Copy)]
pub struct ChannelMap {
    /// Throttle channel index
    pub throttle: usize,
    /// Steering/roll channel index
    pub steering: usize,
    /// Aux 1 (mode switch)
    pub aux1: usize,
    /// Aux 2 (arm/disarm)
    pub aux2: usize,
    /// Aux 3
    pub aux3: usize,
    /// Aux 4
    pub aux4: usize,
}

impl Default for ChannelMap {
    fn default() -> Self {
        // Standard AETR mapping (Aileron, Elevator, Throttle, Rudder)
        Self {
            throttle: 2,  // Channel 3
            steering: 0,  // Channel 1
            aux1: 4,      // Channel 5
            aux2: 5,      // Channel 6
            aux3: 6,      // Channel 7
            aux4: 7,      // Channel 8
        }
    }
}

/// RC server for processing receiver input
#[derive(Debug)]
pub struct RcServer {
    /// Channel mapping
    map: ChannelMap,
    /// Failsafe configuration
    failsafe_config: FailsafeConfig,
    /// Last valid channels
    last_valid: RcChannels,
    /// Time since last valid frame (ms)
    no_signal_ms: u32,
    /// Failsafe active
    in_failsafe: bool,
}

impl RcServer {
    /// Create a new RC server
    pub fn new(map: ChannelMap, failsafe_config: FailsafeConfig) -> Self {
        Self {
            map,
            failsafe_config,
            last_valid: RcChannels::default(),
            no_signal_ms: 0,
            in_failsafe: true, // Start in failsafe until we get signal
        }
    }

    /// Process new channel data
    pub fn process(&mut self, channels: &RcChannels) {
        if channels.is_valid() {
            self.last_valid = *channels;
            self.no_signal_ms = 0;
            self.in_failsafe = false;
        }
    }

    /// Update failsafe timer (call at regular interval)
    pub fn update(&mut self, elapsed_ms: u32) {
        self.no_signal_ms = self.no_signal_ms.saturating_add(elapsed_ms);

        if self.no_signal_ms >= self.failsafe_config.timeout_ms {
            self.in_failsafe = true;
        }
    }

    /// Get current channels (with failsafe applied)
    pub fn channels(&self) -> RcChannels {
        if self.in_failsafe {
            self.get_failsafe_channels()
        } else {
            self.last_valid
        }
    }

    /// Get failsafe channels based on mode
    fn get_failsafe_channels(&self) -> RcChannels {
        let mut channels = match self.failsafe_config.mode {
            FailsafeMode::Hold => self.last_valid,
            FailsafeMode::Neutral => {
                let mut ch = RcChannels::default();
                ch.num_channels = 16;
                ch
            }
            FailsafeMode::Custom => {
                let mut ch = RcChannels::default();
                ch.channels = self.failsafe_config.custom_values;
                ch.num_channels = 16;
                ch
            }
        };
        channels.failsafe = true;
        channels
    }

    /// Check if in failsafe
    pub fn is_failsafe(&self) -> bool {
        self.in_failsafe
    }

    /// Get throttle value [-1.0, 1.0]
    pub fn throttle(&self) -> f32 {
        self.channels().get(self.map.throttle)
    }

    /// Get steering value [-1.0, 1.0]
    pub fn steering(&self) -> f32 {
        self.channels().get(self.map.steering)
    }

    /// Get aux channel value
    pub fn aux(&self, index: usize) -> f32 {
        let ch_index = match index {
            1 => self.map.aux1,
            2 => self.map.aux2,
            3 => self.map.aux3,
            4 => self.map.aux4,
            _ => return 0.0,
        };
        self.channels().get(ch_index)
    }

    /// Check if arm switch is active (aux2 > 0.5)
    pub fn is_armed(&self) -> bool {
        self.aux(2) > 0.5
    }

    /// Get channel map
    pub fn channel_map(&self) -> &ChannelMap {
        &self.map
    }
}

impl Default for RcServer {
    fn default() -> Self {
        Self::new(ChannelMap::default(), FailsafeConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // === SBUS Decoder Tests ===

    fn make_sbus_frame(channels: &[u16; 16], failsafe: bool, frame_lost: bool) -> [u8; 25] {
        let mut frame = [0u8; 25];
        frame[0] = 0x0F; // Start byte

        // Pack 16 channels (11 bits each) into bytes 1-22
        // This is the inverse of the parsing logic
        frame[1] = (channels[0] & 0xFF) as u8;
        frame[2] = ((channels[0] >> 8) | (channels[1] << 3)) as u8;
        frame[3] = ((channels[1] >> 5) | (channels[2] << 6)) as u8;
        frame[4] = ((channels[2] >> 2) & 0xFF) as u8;
        frame[5] = ((channels[2] >> 10) | (channels[3] << 1)) as u8;
        frame[6] = ((channels[3] >> 7) | (channels[4] << 4)) as u8;
        frame[7] = ((channels[4] >> 4) | (channels[5] << 7)) as u8;
        frame[8] = ((channels[5] >> 1) & 0xFF) as u8;
        frame[9] = ((channels[5] >> 9) | (channels[6] << 2)) as u8;
        frame[10] = ((channels[6] >> 6) | (channels[7] << 5)) as u8;
        frame[11] = (channels[7] >> 3) as u8;

        frame[12] = (channels[8] & 0xFF) as u8;
        frame[13] = ((channels[8] >> 8) | (channels[9] << 3)) as u8;
        frame[14] = ((channels[9] >> 5) | (channels[10] << 6)) as u8;
        frame[15] = ((channels[10] >> 2) & 0xFF) as u8;
        frame[16] = ((channels[10] >> 10) | (channels[11] << 1)) as u8;
        frame[17] = ((channels[11] >> 7) | (channels[12] << 4)) as u8;
        frame[18] = ((channels[12] >> 4) | (channels[13] << 7)) as u8;
        frame[19] = ((channels[13] >> 1) & 0xFF) as u8;
        frame[20] = ((channels[13] >> 9) | (channels[14] << 2)) as u8;
        frame[21] = ((channels[14] >> 6) | (channels[15] << 5)) as u8;
        frame[22] = (channels[15] >> 3) as u8;

        // Flags byte
        let mut flags = 0u8;
        if frame_lost { flags |= 0x04; }
        if failsafe { flags |= 0x08; }
        frame[23] = flags;

        frame[24] = 0x00; // End byte
        frame
    }

    #[test]
    fn sbus_decoder_starts_in_wait_state() {
        let decoder = SbusDecoder::new();
        assert_eq!(decoder.state, SbusState::WaitStart);
    }

    #[test]
    fn sbus_decoder_ignores_non_start_bytes() {
        let mut decoder = SbusDecoder::new();
        assert!(decoder.decode(0x00).is_none());
        assert!(decoder.decode(0xFF).is_none());
        assert_eq!(decoder.state, SbusState::WaitStart);
    }

    #[test]
    fn sbus_decoder_accepts_start_byte() {
        let mut decoder = SbusDecoder::new();
        assert!(decoder.decode(0x0F).is_none());
        assert_eq!(decoder.state, SbusState::Receiving);
    }

    #[test]
    fn sbus_decoder_parses_center_values() {
        let mut decoder = SbusDecoder::new();

        // Create frame with all channels at center (992)
        let channels = [992u16; 16];
        let frame = make_sbus_frame(&channels, false, false);

        let mut result = None;
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                result = Some(ch);
            }
        }

        let ch = result.expect("Should decode frame");
        assert_eq!(ch.num_channels, 16);
        assert!(!ch.failsafe);
        assert!(!ch.frame_lost);

        // Center should be close to 0.0
        for i in 0..16 {
            assert!(ch.channels[i].abs() < 0.01, "Channel {} should be ~0, got {}", i, ch.channels[i]);
        }
    }

    #[test]
    fn sbus_decoder_parses_min_max_values() {
        let mut decoder = SbusDecoder::new();

        // Channel 0 at min (172), channel 1 at max (1811)
        let mut channels = [992u16; 16];
        channels[0] = 172;
        channels[1] = 1811;
        let frame = make_sbus_frame(&channels, false, false);

        let mut result = None;
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                result = Some(ch);
            }
        }

        let ch = result.unwrap();
        assert!(ch.channels[0] < -0.9, "Min should be close to -1.0, got {}", ch.channels[0]);
        assert!(ch.channels[1] > 0.9, "Max should be close to 1.0, got {}", ch.channels[1]);
    }

    #[test]
    fn sbus_decoder_detects_failsafe() {
        let mut decoder = SbusDecoder::new();
        let channels = [992u16; 16];
        let frame = make_sbus_frame(&channels, true, false);

        let mut result = None;
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                result = Some(ch);
            }
        }

        let ch = result.unwrap();
        assert!(ch.failsafe);
        assert!(!ch.frame_lost);
    }

    #[test]
    fn sbus_decoder_detects_frame_lost() {
        let mut decoder = SbusDecoder::new();
        let channels = [992u16; 16];
        let frame = make_sbus_frame(&channels, false, true);

        let mut result = None;
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                result = Some(ch);
            }
        }

        let ch = result.unwrap();
        assert!(!ch.failsafe);
        assert!(ch.frame_lost);
    }

    #[test]
    fn sbus_decoder_rejects_bad_end_byte() {
        let mut decoder = SbusDecoder::new();
        let channels = [992u16; 16];
        let mut frame = make_sbus_frame(&channels, false, false);
        frame[24] = 0xFF; // Bad end byte

        let mut result = None;
        for byte in frame {
            if let Some(ch) = decoder.decode(byte) {
                result = Some(ch);
            }
        }

        assert!(result.is_none());
    }

    #[test]
    fn sbus_reset_clears_state() {
        let mut decoder = SbusDecoder::new();
        decoder.decode(0x0F);
        assert_eq!(decoder.state, SbusState::Receiving);

        decoder.reset();
        assert_eq!(decoder.state, SbusState::WaitStart);
    }

    // === Normalization Tests ===

    #[test]
    fn sbus_to_normalized_center() {
        let normalized = sbus_to_normalized(992);
        assert!(normalized.abs() < 0.01);
    }

    #[test]
    fn sbus_to_normalized_min() {
        let normalized = sbus_to_normalized(172);
        assert!(normalized < -0.99);
    }

    #[test]
    fn sbus_to_normalized_max() {
        let normalized = sbus_to_normalized(1811);
        assert!(normalized > 0.99);
    }

    #[test]
    fn normalized_to_sbus_roundtrip() {
        for raw in [172, 500, 992, 1500, 1811] {
            let normalized = sbus_to_normalized(raw);
            let back = normalized_to_sbus(normalized);
            assert!((raw as i32 - back as i32).abs() <= 1, "Roundtrip failed for {}", raw);
        }
    }

    // === RcChannels Tests ===

    #[test]
    fn rc_channels_get_clamps_values() {
        let mut ch = RcChannels::default();
        ch.channels[0] = 2.0;
        ch.channels[1] = -2.0;

        assert_eq!(ch.get(0), 1.0);
        assert_eq!(ch.get(1), -1.0);
    }

    #[test]
    fn rc_channels_get_out_of_range() {
        let ch = RcChannels::default();
        assert_eq!(ch.get(100), 0.0);
    }

    #[test]
    fn rc_channels_is_valid() {
        let mut ch = RcChannels::default();
        assert!(ch.is_valid());

        ch.failsafe = true;
        assert!(!ch.is_valid());

        ch.failsafe = false;
        ch.frame_lost = true;
        assert!(!ch.is_valid());
    }

    // === RC Server Tests ===

    #[test]
    fn rc_server_starts_in_failsafe() {
        let server = RcServer::default();
        assert!(server.is_failsafe());
    }

    #[test]
    fn rc_server_exits_failsafe_on_valid_data() {
        let mut server = RcServer::default();
        let mut ch = RcChannels::default();
        ch.num_channels = 16;

        server.process(&ch);
        assert!(!server.is_failsafe());
    }

    #[test]
    fn rc_server_enters_failsafe_on_timeout() {
        let mut server = RcServer::default();
        let mut ch = RcChannels::default();
        ch.num_channels = 16;

        server.process(&ch);
        assert!(!server.is_failsafe());

        // Simulate timeout
        server.update(server.failsafe_config.timeout_ms + 1);
        assert!(server.is_failsafe());
    }

    #[test]
    fn rc_server_failsafe_hold_mode() {
        let mut config = FailsafeConfig::default();
        config.mode = FailsafeMode::Hold;
        let mut server = RcServer::new(ChannelMap::default(), config);

        let mut ch = RcChannels::default();
        ch.channels[0] = 0.5; // Set steering
        ch.num_channels = 16;

        server.process(&ch);
        server.update(1000); // Trigger failsafe

        let fs_ch = server.channels();
        assert!(fs_ch.failsafe);
        assert_eq!(fs_ch.channels[0], 0.5); // Should hold last value
    }

    #[test]
    fn rc_server_failsafe_neutral_mode() {
        let mut config = FailsafeConfig::default();
        config.mode = FailsafeMode::Neutral;
        let mut server = RcServer::new(ChannelMap::default(), config);

        let mut ch = RcChannels::default();
        ch.channels[0] = 0.5;
        ch.num_channels = 16;

        server.process(&ch);
        server.update(1000);

        let fs_ch = server.channels();
        assert!(fs_ch.failsafe);
        assert_eq!(fs_ch.channels[0], 0.0); // Should be neutral
    }

    #[test]
    fn rc_server_throttle_steering() {
        let mut server = RcServer::default();
        let mut ch = RcChannels::default();
        ch.channels[0] = 0.5;  // Steering (ch1)
        ch.channels[2] = 0.8;  // Throttle (ch3)
        ch.num_channels = 16;

        server.process(&ch);

        assert!((server.steering() - 0.5).abs() < 0.01);
        assert!((server.throttle() - 0.8).abs() < 0.01);
    }

    #[test]
    fn rc_server_arm_switch() {
        let mut server = RcServer::default();
        let mut ch = RcChannels::default();
        ch.channels[5] = -1.0; // Aux2 low = disarmed
        ch.num_channels = 16;

        server.process(&ch);
        assert!(!server.is_armed());

        ch.channels[5] = 1.0; // Aux2 high = armed
        server.process(&ch);
        assert!(server.is_armed());
    }

    // === CRSF CRC Tests ===

    #[test]
    fn crsf_crc8_empty() {
        assert_eq!(crsf_crc8(&[]), 0x00);
    }

    #[test]
    fn crsf_crc8_known_value() {
        // Known test vector
        let data = [0x16, 0x00, 0x00, 0x00];
        let crc = crsf_crc8(&data);
        assert!(crc != 0); // Just verify it computes something
    }

    // === CRSF Decoder Tests ===

    #[test]
    fn crsf_decoder_starts_in_wait_state() {
        let decoder = CrsfDecoder::new();
        assert_eq!(decoder.state, CrsfState::WaitSync);
    }

    #[test]
    fn crsf_decoder_accepts_address() {
        let mut decoder = CrsfDecoder::new();
        assert!(decoder.decode(0xC8).is_none());
        assert_eq!(decoder.state, CrsfState::WaitLength);
    }

    #[test]
    fn crsf_decoder_rejects_bad_length() {
        let mut decoder = CrsfDecoder::new();
        decoder.decode(0xC8); // Address
        decoder.decode(0x00); // Length 0 - invalid
        assert_eq!(decoder.state, CrsfState::WaitSync);
    }

    #[test]
    fn crsf_reset_clears_state() {
        let mut decoder = CrsfDecoder::new();
        decoder.decode(0xC8);
        assert_eq!(decoder.state, CrsfState::WaitLength);

        decoder.reset();
        assert_eq!(decoder.state, CrsfState::WaitSync);
    }

    // === Channel Map Tests ===

    #[test]
    fn channel_map_default_aetr() {
        let map = ChannelMap::default();
        assert_eq!(map.steering, 0);  // Aileron/Roll = Ch1
        assert_eq!(map.throttle, 2);  // Throttle = Ch3
    }

    // === Failsafe Config Tests ===

    #[test]
    fn failsafe_config_default() {
        let config = FailsafeConfig::default();
        assert_eq!(config.mode, FailsafeMode::Hold);
        assert_eq!(config.timeout_ms, 500);
    }
}
