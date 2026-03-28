//! GPS Server
//!
//! Decodes GPS protocols and provides position/velocity data.
//!
//! Supported protocols:
//! - NMEA 0183 (ASCII) - Universal, works with all GPS modules
//! - UBX (binary) - u-blox specific, more efficient
//!
//! Features:
//! - Position (lat/lon/alt)
//! - Velocity (speed/course)
//! - Fix quality and satellite info
//! - HDOP (horizontal dilution of precision)

/// Maximum NMEA sentence length (including $, *, checksum, CR, LF)
pub const NMEA_MAX_LENGTH: usize = 82;

/// UBX NAV-PVT payload length
pub const UBX_NAV_PVT_LENGTH: usize = 92;

/// GPS fix quality
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum GpsFix {
    /// No fix
    #[default]
    NoFix = 0,
    /// 2D fix (no altitude)
    Fix2D = 1,
    /// 3D fix
    Fix3D = 2,
    /// Differential GPS
    DGps = 3,
    /// RTK fixed
    Rtk = 4,
    /// RTK float
    RtkFloat = 5,
}

impl GpsFix {
    /// Check if we have a usable fix
    pub fn has_fix(&self) -> bool {
        matches!(self, GpsFix::Fix2D | GpsFix::Fix3D | GpsFix::DGps | GpsFix::Rtk | GpsFix::RtkFloat)
    }

    /// Check if we have a 3D fix
    pub fn is_3d(&self) -> bool {
        matches!(self, GpsFix::Fix3D | GpsFix::DGps | GpsFix::Rtk | GpsFix::RtkFloat)
    }
}

/// Parsed GPS data
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsData {
    /// Latitude in degrees (positive = North)
    pub latitude: f64,
    /// Longitude in degrees (positive = East)
    pub longitude: f64,
    /// Altitude above mean sea level in meters
    pub altitude: f32,
    /// Ground speed in m/s
    pub speed: f32,
    /// Course over ground in degrees (0-360, 0 = North)
    pub course: f32,
    /// Fix quality
    pub fix: GpsFix,
    /// Horizontal dilution of precision
    pub hdop: f32,
    /// Number of satellites used
    pub satellites: u8,
    /// Timestamp in milliseconds (GPS time of day)
    pub timestamp_ms: u32,
    /// Data validity flag
    pub valid: bool,
}

impl GpsData {
    /// Check if position data is valid and usable
    pub fn is_valid(&self) -> bool {
        self.valid && self.fix.has_fix()
    }
}

// ============================================================================
// NMEA Decoder
// ============================================================================

/// NMEA decoder state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NmeaState {
    /// Waiting for '$' start character
    WaitDollar,
    /// Reading sentence data (between $ and *)
    ReadingData,
    /// Reading first checksum hex digit
    ReadingCrc1,
    /// Reading second checksum hex digit
    ReadingCrc2,
    /// Waiting for CR
    WaitCr,
    /// Waiting for LF
    WaitLf,
}

/// NMEA sentence type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmeaSentence {
    /// GGA - Global Positioning System Fix Data
    Gga,
    /// RMC - Recommended Minimum Navigation Information
    Rmc,
    /// GSA - GPS DOP and Active Satellites
    Gsa,
    /// VTG - Course Over Ground and Ground Speed
    Vtg,
    /// Unknown sentence type
    Unknown,
}

/// NMEA protocol decoder
///
/// Parses NMEA 0183 sentences byte-by-byte and extracts GPS data.
///
/// Supported sentences:
/// - GGA: Position, altitude, fix quality, satellites, HDOP
/// - RMC: Position, speed, course, validity
/// - GSA: DOP values, fix type
/// - VTG: Course and speed (backup)
#[derive(Debug)]
pub struct NmeaDecoder {
    /// Sentence buffer (excluding $ and *)
    buffer: [u8; NMEA_MAX_LENGTH],
    /// Current buffer index
    index: usize,
    /// Current state
    state: NmeaState,
    /// Running XOR checksum
    checksum: u8,
    /// Received checksum value
    received_crc: u8,
    /// Accumulated GPS data
    data: GpsData,
    /// Flags for which data has been received
    has_position: bool,
    has_velocity: bool,
}

impl NmeaDecoder {
    /// Create a new NMEA decoder
    pub fn new() -> Self {
        Self {
            buffer: [0; NMEA_MAX_LENGTH],
            index: 0,
            state: NmeaState::WaitDollar,
            checksum: 0,
            received_crc: 0,
            data: GpsData::default(),
            has_position: false,
            has_velocity: false,
        }
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.index = 0;
        self.state = NmeaState::WaitDollar;
        self.checksum = 0;
        self.received_crc = 0;
    }

    /// Get last parsed data
    pub fn last_data(&self) -> &GpsData {
        &self.data
    }

    /// Feed a byte to the decoder
    ///
    /// Returns `Some(GpsData)` when a complete valid sentence is parsed
    /// that provides new position or velocity information.
    pub fn decode(&mut self, byte: u8) -> Option<GpsData> {
        match self.state {
            NmeaState::WaitDollar => {
                if byte == b'$' {
                    self.index = 0;
                    self.checksum = 0;
                    self.state = NmeaState::ReadingData;
                }
                None
            }
            NmeaState::ReadingData => {
                if byte == b'*' {
                    self.state = NmeaState::ReadingCrc1;
                    None
                } else if byte == b'$' {
                    // Restart on unexpected $
                    self.index = 0;
                    self.checksum = 0;
                    None
                } else if self.index < NMEA_MAX_LENGTH {
                    self.buffer[self.index] = byte;
                    self.index += 1;
                    self.checksum ^= byte;
                    None
                } else {
                    // Buffer overflow, reset
                    self.reset();
                    None
                }
            }
            NmeaState::ReadingCrc1 => {
                if let Some(val) = hex_digit(byte) {
                    self.received_crc = val << 4;
                    self.state = NmeaState::ReadingCrc2;
                } else {
                    self.reset();
                }
                None
            }
            NmeaState::ReadingCrc2 => {
                if let Some(val) = hex_digit(byte) {
                    self.received_crc |= val;
                    self.state = NmeaState::WaitCr;
                } else {
                    self.reset();
                }
                None
            }
            NmeaState::WaitCr => {
                if byte == b'\r' {
                    self.state = NmeaState::WaitLf;
                } else {
                    self.reset();
                }
                None
            }
            NmeaState::WaitLf => {
                self.state = NmeaState::WaitDollar;

                if byte == b'\n' && self.checksum == self.received_crc {
                    // Valid sentence, parse it
                    self.parse_sentence()
                } else {
                    None
                }
            }
        }
    }

    /// Parse the buffered sentence
    fn parse_sentence(&mut self) -> Option<GpsData> {
        let len = self.index;

        // Determine sentence type from talker ID + sentence type
        // Format: XXYYY where XX is talker (GP, GN, GL, etc.) and YYY is sentence
        if len < 6 {
            return None;
        }

        let sentence_type = match &self.buffer[2..6] {
            b"GGA," => NmeaSentence::Gga,
            b"RMC," => NmeaSentence::Rmc,
            b"GSA," => NmeaSentence::Gsa,
            b"VTG," => NmeaSentence::Vtg,
            _ => NmeaSentence::Unknown,
        };

        match sentence_type {
            NmeaSentence::Gga => self.parse_gga(len),
            NmeaSentence::Rmc => self.parse_rmc(len),
            NmeaSentence::Gsa => self.parse_gsa(len),
            NmeaSentence::Vtg => self.parse_vtg(len),
            NmeaSentence::Unknown => None,
        }
    }

    /// Parse GGA sentence (Global Positioning System Fix Data)
    ///
    /// Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    fn parse_gga(&mut self, len: usize) -> Option<GpsData> {
        let mut fields = FieldIterator::new(&self.buffer[..len]);

        // Skip talker+sentence ID
        fields.next();

        // Time (hhmmss.ss)
        if let Some(time_field) = fields.next() {
            if let Some(ms) = parse_time(time_field) {
                self.data.timestamp_ms = ms;
            }
        }

        // Latitude (llll.ll)
        let lat_field = fields.next();
        // N/S indicator
        let lat_dir = fields.next();

        // Longitude (yyyyy.yy)
        let lon_field = fields.next();
        // E/W indicator
        let lon_dir = fields.next();

        // Parse position
        if let (Some(lat), Some(lat_d), Some(lon), Some(lon_d)) = (lat_field, lat_dir, lon_field, lon_dir) {
            if let (Some(latitude), Some(longitude)) = (parse_lat(lat, lat_d), parse_lon(lon, lon_d)) {
                self.data.latitude = latitude;
                self.data.longitude = longitude;
                self.has_position = true;
            }
        }

        // Fix quality (0=invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float RTK)
        if let Some(fix_field) = fields.next() {
            self.data.fix = match fix_field {
                b"0" => GpsFix::NoFix,
                b"1" => GpsFix::Fix3D,
                b"2" => GpsFix::DGps,
                b"4" => GpsFix::Rtk,
                b"5" => GpsFix::RtkFloat,
                _ => self.data.fix,
            };
        }

        // Number of satellites
        if let Some(sat_field) = fields.next() {
            if let Some(sats) = parse_u8(sat_field) {
                self.data.satellites = sats;
            }
        }

        // HDOP
        if let Some(hdop_field) = fields.next() {
            if let Some(hdop) = parse_f32(hdop_field) {
                self.data.hdop = hdop;
            }
        }

        // Altitude above MSL
        if let Some(alt_field) = fields.next() {
            if let Some(alt) = parse_f32(alt_field) {
                self.data.altitude = alt;
            }
        }

        self.data.valid = self.data.fix.has_fix();

        if self.has_position {
            Some(self.data)
        } else {
            None
        }
    }

    /// Parse RMC sentence (Recommended Minimum Navigation Information)
    ///
    /// Format: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    fn parse_rmc(&mut self, len: usize) -> Option<GpsData> {
        let mut fields = FieldIterator::new(&self.buffer[..len]);

        // Skip talker+sentence ID
        fields.next();

        // Time
        if let Some(time_field) = fields.next() {
            if let Some(ms) = parse_time(time_field) {
                self.data.timestamp_ms = ms;
            }
        }

        // Status (A=active, V=void)
        let status = fields.next();
        let is_valid = status == Some(b"A");

        // Latitude
        let lat_field = fields.next();
        let lat_dir = fields.next();

        // Longitude
        let lon_field = fields.next();
        let lon_dir = fields.next();

        // Parse position
        if is_valid {
            if let (Some(lat), Some(lat_d), Some(lon), Some(lon_d)) = (lat_field, lat_dir, lon_field, lon_dir) {
                if let (Some(latitude), Some(longitude)) = (parse_lat(lat, lat_d), parse_lon(lon, lon_d)) {
                    self.data.latitude = latitude;
                    self.data.longitude = longitude;
                    self.has_position = true;
                }
            }
        }

        // Speed over ground (knots)
        if let Some(speed_field) = fields.next() {
            if let Some(speed_knots) = parse_f32(speed_field) {
                // Convert knots to m/s (1 knot = 0.514444 m/s)
                self.data.speed = speed_knots * 0.514444;
                self.has_velocity = true;
            }
        }

        // Course over ground (degrees true)
        if let Some(course_field) = fields.next() {
            if let Some(course) = parse_f32(course_field) {
                self.data.course = course;
            }
        }

        self.data.valid = is_valid && self.data.fix.has_fix();

        if self.has_position || self.has_velocity {
            Some(self.data)
        } else {
            None
        }
    }

    /// Parse GSA sentence (GPS DOP and Active Satellites)
    fn parse_gsa(&mut self, len: usize) -> Option<GpsData> {
        let mut fields = FieldIterator::new(&self.buffer[..len]);

        // Skip talker+sentence ID
        fields.next();

        // Mode (M=manual, A=automatic)
        fields.next();

        // Fix type (1=no fix, 2=2D, 3=3D)
        if let Some(fix_field) = fields.next() {
            self.data.fix = match fix_field {
                b"1" => GpsFix::NoFix,
                b"2" => GpsFix::Fix2D,
                b"3" => GpsFix::Fix3D,
                _ => self.data.fix,
            };
        }

        // Skip PRNs (12 fields)
        for _ in 0..12 {
            fields.next();
        }

        // PDOP (skip)
        fields.next();

        // HDOP
        if let Some(hdop_field) = fields.next() {
            if let Some(hdop) = parse_f32(hdop_field) {
                self.data.hdop = hdop;
            }
        }

        self.data.valid = self.data.fix.has_fix();

        // GSA alone doesn't provide position, return None
        None
    }

    /// Parse VTG sentence (Course Over Ground and Ground Speed)
    fn parse_vtg(&mut self, len: usize) -> Option<GpsData> {
        let mut fields = FieldIterator::new(&self.buffer[..len]);

        // Skip talker+sentence ID
        fields.next();

        // Course true
        if let Some(course_field) = fields.next() {
            if let Some(course) = parse_f32(course_field) {
                self.data.course = course;
            }
        }

        // Skip T, course magnetic, M
        fields.next();
        fields.next();
        fields.next();

        // Speed in knots
        if let Some(speed_field) = fields.next() {
            if let Some(speed_knots) = parse_f32(speed_field) {
                self.data.speed = speed_knots * 0.514444;
                self.has_velocity = true;
            }
        }

        // Skip N, speed in km/h, K, mode
        // ...

        if self.has_velocity {
            Some(self.data)
        } else {
            None
        }
    }
}

impl Default for NmeaDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert hex digit character to value
fn hex_digit(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'A'..=b'F' => Some(c - b'A' + 10),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

/// Iterator over comma-separated NMEA fields
struct FieldIterator<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> FieldIterator<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }
}

impl<'a> Iterator for FieldIterator<'a> {
    type Item = &'a [u8];

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.data.len() {
            return None;
        }

        let start = self.pos;
        while self.pos < self.data.len() && self.data[self.pos] != b',' {
            self.pos += 1;
        }

        let end = self.pos;
        self.pos += 1; // Skip comma

        Some(&self.data[start..end])
    }
}

/// Parse NMEA time field (hhmmss.ss) to milliseconds since midnight
fn parse_time(field: &[u8]) -> Option<u32> {
    if field.len() < 6 {
        return None;
    }

    let hours = parse_u8(&field[0..2])?;
    let minutes = parse_u8(&field[2..4])?;
    let seconds = parse_u8(&field[4..6])?;

    let mut ms = (hours as u32 * 3600 + minutes as u32 * 60 + seconds as u32) * 1000;

    // Parse fractional seconds if present
    if field.len() > 7 && field[6] == b'.' {
        if let Some(frac) = parse_u8(&field[7..9.min(field.len())]) {
            ms += frac as u32 * 10;
        }
    }

    Some(ms)
}

/// Parse latitude (DDMM.MMMM format) with N/S indicator
fn parse_lat(field: &[u8], dir: &[u8]) -> Option<f64> {
    if field.len() < 4 {
        return None;
    }

    let degrees = parse_u8(&field[0..2])? as f64;
    let minutes = parse_f64(&field[2..])?;
    let mut lat = degrees + minutes / 60.0;

    if dir == b"S" {
        lat = -lat;
    }

    Some(lat)
}

/// Parse longitude (DDDMM.MMMM format) with E/W indicator
fn parse_lon(field: &[u8], dir: &[u8]) -> Option<f64> {
    if field.len() < 5 {
        return None;
    }

    let degrees = parse_u8_3(&field[0..3])? as f64;
    let minutes = parse_f64(&field[3..])?;
    let mut lon = degrees + minutes / 60.0;

    if dir == b"W" {
        lon = -lon;
    }

    Some(lon)
}

/// Parse u8 from 2-digit ASCII
fn parse_u8(field: &[u8]) -> Option<u8> {
    if field.len() < 2 {
        return None;
    }
    let d1 = field[0].wrapping_sub(b'0');
    let d2 = field[1].wrapping_sub(b'0');
    if d1 > 9 || d2 > 9 {
        return None;
    }
    Some(d1 * 10 + d2)
}

/// Parse u8 from 3-digit ASCII (for longitude degrees)
fn parse_u8_3(field: &[u8]) -> Option<u16> {
    if field.len() < 3 {
        return None;
    }
    let d1 = field[0].wrapping_sub(b'0') as u16;
    let d2 = field[1].wrapping_sub(b'0') as u16;
    let d3 = field[2].wrapping_sub(b'0') as u16;
    if d1 > 9 || d2 > 9 || d3 > 9 {
        return None;
    }
    Some(d1 * 100 + d2 * 10 + d3)
}

/// Parse f32 from ASCII (simple, no exponent support)
fn parse_f32(field: &[u8]) -> Option<f32> {
    parse_f64(field).map(|v| v as f32)
}

/// Parse f64 from ASCII (simple, no exponent support)
fn parse_f64(field: &[u8]) -> Option<f64> {
    if field.is_empty() {
        return None;
    }

    let mut result: f64 = 0.0;
    let mut decimal_place: Option<f64> = None;
    let mut negative = false;
    let mut has_digit = false;

    for &c in field {
        match c {
            b'-' if !has_digit => negative = true,
            b'.' if decimal_place.is_none() => decimal_place = Some(0.1),
            b'0'..=b'9' => {
                has_digit = true;
                let digit = (c - b'0') as f64;
                if let Some(ref mut place) = decimal_place {
                    result += digit * *place;
                    *place *= 0.1;
                } else {
                    result = result * 10.0 + digit;
                }
            }
            _ => return None,
        }
    }

    if !has_digit {
        return None;
    }

    if negative {
        result = -result;
    }

    Some(result)
}

// ============================================================================
// UBX Protocol Decoder
// ============================================================================

/// UBX sync bytes
const UBX_SYNC1: u8 = 0xB5;
const UBX_SYNC2: u8 = 0x62;

/// UBX message classes
const UBX_CLASS_NAV: u8 = 0x01;
const UBX_CLASS_CFG: u8 = 0x06;
const UBX_CLASS_ACK: u8 = 0x05;

/// UBX message IDs
const UBX_NAV_PVT: u8 = 0x07;
const UBX_CFG_RATE: u8 = 0x08;
const UBX_CFG_MSG: u8 = 0x01;
const UBX_ACK_ACK: u8 = 0x01;
const UBX_ACK_NAK: u8 = 0x00;

/// Maximum UBX payload size we support
const UBX_MAX_PAYLOAD: usize = 128;

/// UBX decoder state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UbxState {
    /// Waiting for first sync byte (0xB5)
    WaitSync1,
    /// Waiting for second sync byte (0x62)
    WaitSync2,
    /// Reading message class
    WaitClass,
    /// Reading message ID
    WaitId,
    /// Reading payload length (low byte)
    WaitLen1,
    /// Reading payload length (high byte)
    WaitLen2,
    /// Receiving payload bytes
    Receiving,
    /// Reading first checksum byte
    WaitCkA,
    /// Reading second checksum byte
    WaitCkB,
}

/// UBX protocol decoder
///
/// Parses u-blox binary protocol messages byte-by-byte.
///
/// Supported messages:
/// - NAV-PVT (0x01 0x07): Position, velocity, and time solution
#[derive(Debug)]
pub struct UbxDecoder {
    /// Payload buffer
    buffer: [u8; UBX_MAX_PAYLOAD],
    /// Current buffer index
    index: usize,
    /// Current state
    state: UbxState,
    /// Message class
    msg_class: u8,
    /// Message ID
    msg_id: u8,
    /// Payload length
    payload_len: u16,
    /// Running checksum A (Fletcher)
    ck_a: u8,
    /// Running checksum B (Fletcher)
    ck_b: u8,
    /// Received checksum A
    recv_ck_a: u8,
    /// Accumulated GPS data
    data: GpsData,
}

impl UbxDecoder {
    /// Create a new UBX decoder
    pub fn new() -> Self {
        Self {
            buffer: [0; UBX_MAX_PAYLOAD],
            index: 0,
            state: UbxState::WaitSync1,
            msg_class: 0,
            msg_id: 0,
            payload_len: 0,
            ck_a: 0,
            ck_b: 0,
            recv_ck_a: 0,
            data: GpsData::default(),
        }
    }

    /// Reset decoder state
    pub fn reset(&mut self) {
        self.index = 0;
        self.state = UbxState::WaitSync1;
        self.ck_a = 0;
        self.ck_b = 0;
    }

    /// Get last parsed data
    pub fn last_data(&self) -> &GpsData {
        &self.data
    }

    /// Update Fletcher checksum with a byte
    fn update_checksum(&mut self, byte: u8) {
        self.ck_a = self.ck_a.wrapping_add(byte);
        self.ck_b = self.ck_b.wrapping_add(self.ck_a);
    }

    /// Feed a byte to the decoder
    ///
    /// Returns `Some(GpsData)` when a complete valid NAV-PVT message is parsed.
    pub fn decode(&mut self, byte: u8) -> Option<GpsData> {
        match self.state {
            UbxState::WaitSync1 => {
                if byte == UBX_SYNC1 {
                    self.state = UbxState::WaitSync2;
                }
                None
            }
            UbxState::WaitSync2 => {
                if byte == UBX_SYNC2 {
                    self.ck_a = 0;
                    self.ck_b = 0;
                    self.state = UbxState::WaitClass;
                } else if byte == UBX_SYNC1 {
                    // Stay in WaitSync2 (could be B5 B5 62 sequence)
                } else {
                    self.state = UbxState::WaitSync1;
                }
                None
            }
            UbxState::WaitClass => {
                self.msg_class = byte;
                self.update_checksum(byte);
                self.state = UbxState::WaitId;
                None
            }
            UbxState::WaitId => {
                self.msg_id = byte;
                self.update_checksum(byte);
                self.state = UbxState::WaitLen1;
                None
            }
            UbxState::WaitLen1 => {
                self.payload_len = byte as u16;
                self.update_checksum(byte);
                self.state = UbxState::WaitLen2;
                None
            }
            UbxState::WaitLen2 => {
                self.payload_len |= (byte as u16) << 8;
                self.update_checksum(byte);

                if self.payload_len == 0 {
                    self.state = UbxState::WaitCkA;
                } else if self.payload_len as usize > UBX_MAX_PAYLOAD {
                    // Payload too large, reset
                    self.reset();
                } else {
                    self.index = 0;
                    self.state = UbxState::Receiving;
                }
                None
            }
            UbxState::Receiving => {
                self.buffer[self.index] = byte;
                self.index += 1;
                self.update_checksum(byte);

                if self.index >= self.payload_len as usize {
                    self.state = UbxState::WaitCkA;
                }
                None
            }
            UbxState::WaitCkA => {
                self.recv_ck_a = byte;
                self.state = UbxState::WaitCkB;
                None
            }
            UbxState::WaitCkB => {
                self.state = UbxState::WaitSync1;

                if self.ck_a == self.recv_ck_a && self.ck_b == byte {
                    // Valid message, parse it
                    self.parse_message()
                } else {
                    None
                }
            }
        }
    }

    /// Parse the received UBX message
    fn parse_message(&mut self) -> Option<GpsData> {
        if self.msg_class == UBX_CLASS_NAV && self.msg_id == UBX_NAV_PVT {
            self.parse_nav_pvt()
        } else {
            None
        }
    }

    /// Parse NAV-PVT message (92 bytes payload)
    ///
    /// See u-blox protocol specification for field layout.
    fn parse_nav_pvt(&mut self) -> Option<GpsData> {
        if self.payload_len < 92 {
            return None;
        }

        let buf = &self.buffer;

        // Byte 0-3: iTOW (GPS time of week, ms)
        let _itow = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);

        // Byte 4-5: year
        // Byte 6: month
        // Byte 7: day
        // Byte 8: hour
        // Byte 9: min
        // Byte 10: sec

        // Calculate timestamp (simplified: just use hour:min:sec)
        let hour = buf[8] as u32;
        let min = buf[9] as u32;
        let sec = buf[10] as u32;
        self.data.timestamp_ms = (hour * 3600 + min * 60 + sec) * 1000;

        // Byte 11: valid flags
        let valid_flags = buf[11];
        let date_valid = (valid_flags & 0x01) != 0;
        let time_valid = (valid_flags & 0x02) != 0;

        // Byte 20: fixType (0=no fix, 2=2D, 3=3D)
        self.data.fix = match buf[20] {
            0 => GpsFix::NoFix,
            1 => GpsFix::NoFix, // Dead reckoning only
            2 => GpsFix::Fix2D,
            3 => GpsFix::Fix3D,
            4 => GpsFix::DGps, // GNSS + dead reckoning
            5 => GpsFix::NoFix, // Time only
            _ => GpsFix::NoFix,
        };

        // Byte 21: flags (check gnssFixOK)
        let flags = buf[21];
        let gnss_fix_ok = (flags & 0x01) != 0;

        // Byte 23: numSV
        self.data.satellites = buf[23];

        // Byte 24-27: lon (1e-7 degrees)
        let lon_raw = i32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]);
        self.data.longitude = lon_raw as f64 / 1e7;

        // Byte 28-31: lat (1e-7 degrees)
        let lat_raw = i32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]);
        self.data.latitude = lat_raw as f64 / 1e7;

        // Byte 36-39: hMSL (mm above mean sea level)
        let hmsl = i32::from_le_bytes([buf[36], buf[37], buf[38], buf[39]]);
        self.data.altitude = hmsl as f32 / 1000.0;

        // Byte 60-63: gSpeed (mm/s ground speed)
        let gspeed = i32::from_le_bytes([buf[60], buf[61], buf[62], buf[63]]);
        self.data.speed = gspeed as f32 / 1000.0;

        // Byte 64-67: headMot (1e-5 degrees, heading of motion)
        let head_mot = i32::from_le_bytes([buf[64], buf[65], buf[66], buf[67]]);
        let course = libm::fmodf(head_mot as f32 / 1e5, 360.0);
        self.data.course = if course < 0.0 { course + 360.0 } else { course };

        // Byte 76-77: pDOP (0.01 scale)
        let pdop = u16::from_le_bytes([buf[76], buf[77]]);
        // Use pDOP as approximation for HDOP
        self.data.hdop = pdop as f32 / 100.0;

        self.data.valid = gnss_fix_ok && self.data.fix.has_fix() && date_valid && time_valid;

        Some(self.data)
    }
}

impl Default for UbxDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// UBX Configuration Builder
// ============================================================================

/// Build UBX configuration commands
pub struct UbxConfigBuilder;

impl UbxConfigBuilder {
    /// Build UBX-CFG-RATE message to set measurement rate
    ///
    /// Returns the number of bytes written to the buffer.
    ///
    /// # Arguments
    /// * `rate_ms` - Measurement period in milliseconds (e.g., 100 for 10Hz)
    /// * `buf` - Output buffer (must be at least 14 bytes)
    pub fn build_cfg_rate(rate_ms: u16, buf: &mut [u8]) -> usize {
        if buf.len() < 14 {
            return 0;
        }

        // Header
        buf[0] = UBX_SYNC1;
        buf[1] = UBX_SYNC2;
        buf[2] = UBX_CLASS_CFG;
        buf[3] = UBX_CFG_RATE;
        buf[4] = 6; // Payload length low
        buf[5] = 0; // Payload length high

        // Payload
        buf[6] = (rate_ms & 0xFF) as u8;         // measRate low
        buf[7] = ((rate_ms >> 8) & 0xFF) as u8;  // measRate high
        buf[8] = 1;  // navRate (cycles) - always 1
        buf[9] = 0;
        buf[10] = 1; // timeRef - GPS time
        buf[11] = 0;

        // Calculate checksum over class, id, length, payload
        let (ck_a, ck_b) = Self::checksum(&buf[2..12]);
        buf[12] = ck_a;
        buf[13] = ck_b;

        14
    }

    /// Build UBX-CFG-MSG message to enable/disable a message
    ///
    /// Returns the number of bytes written to the buffer.
    ///
    /// # Arguments
    /// * `msg_class` - Message class
    /// * `msg_id` - Message ID
    /// * `rate` - Output rate (0 to disable, 1+ to enable)
    /// * `buf` - Output buffer (must be at least 11 bytes)
    pub fn build_cfg_msg(msg_class: u8, msg_id: u8, rate: u8, buf: &mut [u8]) -> usize {
        if buf.len() < 11 {
            return 0;
        }

        // Header
        buf[0] = UBX_SYNC1;
        buf[1] = UBX_SYNC2;
        buf[2] = UBX_CLASS_CFG;
        buf[3] = UBX_CFG_MSG;
        buf[4] = 3; // Payload length low
        buf[5] = 0; // Payload length high

        // Payload
        buf[6] = msg_class;
        buf[7] = msg_id;
        buf[8] = rate;

        // Checksum
        let (ck_a, ck_b) = Self::checksum(&buf[2..9]);
        buf[9] = ck_a;
        buf[10] = ck_b;

        11
    }

    /// Calculate Fletcher checksum
    fn checksum(data: &[u8]) -> (u8, u8) {
        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;
        for &byte in data {
            ck_a = ck_a.wrapping_add(byte);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        (ck_a, ck_b)
    }
}

// ============================================================================
// GPS Server
// ============================================================================

/// GPS server configuration
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    /// Timeout for valid fix (ms). After this time without data, fix is invalid.
    pub timeout_ms: u32,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            timeout_ms: 2000, // 2 seconds
        }
    }
}

/// GPS server that combines NMEA and UBX decoders
///
/// Automatically detects protocol and parses incoming bytes.
/// Maintains the latest GPS state and handles timeouts.
pub struct GpsServer {
    nmea: NmeaDecoder,
    ubx: UbxDecoder,
    config: GpsConfig,
    last_data: GpsData,
    time_since_update_ms: u32,
}

impl GpsServer {
    /// Create a new GPS server
    pub fn new(config: GpsConfig) -> Self {
        Self {
            nmea: NmeaDecoder::new(),
            ubx: UbxDecoder::new(),
            config,
            last_data: GpsData::default(),
            time_since_update_ms: u32::MAX, // Start as "no data"
        }
    }

    /// Process a byte from the GPS receiver
    ///
    /// The byte is fed to both NMEA and UBX decoders. If either produces
    /// valid GPS data, it updates the internal state and returns the data.
    pub fn process_byte(&mut self, byte: u8) -> Option<GpsData> {
        // Try NMEA first (more common default)
        if let Some(data) = self.nmea.decode(byte) {
            self.last_data = data;
            self.time_since_update_ms = 0;
            return Some(data);
        }

        // Try UBX
        if let Some(data) = self.ubx.decode(byte) {
            self.last_data = data;
            self.time_since_update_ms = 0;
            return Some(data);
        }

        None
    }

    /// Update timeout tracking
    ///
    /// Call this periodically (e.g., every control loop iteration) with
    /// the elapsed time in milliseconds since the last call.
    pub fn update(&mut self, elapsed_ms: u32) {
        self.time_since_update_ms = self.time_since_update_ms.saturating_add(elapsed_ms);

        // Mark data as invalid if timeout exceeded
        if self.time_since_update_ms > self.config.timeout_ms {
            self.last_data.valid = false;
        }
    }

    /// Get the last received GPS data
    pub fn last_data(&self) -> &GpsData {
        &self.last_data
    }

    /// Check if we have a valid fix
    pub fn has_fix(&self) -> bool {
        self.last_data.valid && self.last_data.fix.has_fix()
    }

    /// Check if we have a 3D fix
    pub fn has_3d_fix(&self) -> bool {
        self.last_data.valid && self.last_data.fix.is_3d()
    }

    /// Get time since last valid update (ms)
    pub fn time_since_update(&self) -> u32 {
        self.time_since_update_ms
    }

    /// Reset the server state
    pub fn reset(&mut self) {
        self.nmea.reset();
        self.ubx.reset();
        self.last_data = GpsData::default();
        self.time_since_update_ms = u32::MAX;
    }
}

impl Default for GpsServer {
    fn default() -> Self {
        Self::new(GpsConfig::default())
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Feed a string to the decoder byte by byte
    fn feed_string(decoder: &mut NmeaDecoder, s: &str) -> Option<GpsData> {
        let mut result = None;
        for byte in s.bytes() {
            if let Some(data) = decoder.decode(byte) {
                result = Some(data);
            }
        }
        result
    }

    #[test]
    fn test_parse_f64() {
        assert_eq!(parse_f64(b"123"), Some(123.0));
        assert_eq!(parse_f64(b"123.456"), Some(123.456));
        assert_eq!(parse_f64(b"-123.456"), Some(-123.456));
        assert_eq!(parse_f64(b"0.5"), Some(0.5));
        assert_eq!(parse_f64(b""), None);
        assert_eq!(parse_f64(b"."), None);
    }

    #[test]
    fn test_parse_time() {
        // 12:35:19.00 = 45319000 ms
        assert_eq!(parse_time(b"123519"), Some(45319000));
        assert_eq!(parse_time(b"123519.00"), Some(45319000));
        assert_eq!(parse_time(b"000000"), Some(0));
        assert_eq!(parse_time(b"235959"), Some(86399000));
    }

    #[test]
    fn test_parse_lat() {
        // 48 degrees 7.038 minutes N = 48.1173 degrees
        let lat = parse_lat(b"4807.038", b"N").unwrap();
        assert!((lat - 48.1173).abs() < 0.0001);

        // Same but South
        let lat_s = parse_lat(b"4807.038", b"S").unwrap();
        assert!((lat_s - (-48.1173)).abs() < 0.0001);
    }

    #[test]
    fn test_parse_lon() {
        // 11 degrees 31.000 minutes E = 11.516667 degrees
        let lon = parse_lon(b"01131.000", b"E").unwrap();
        assert!((lon - 11.516667).abs() < 0.0001);

        // West
        let lon_w = parse_lon(b"01131.000", b"W").unwrap();
        assert!((lon_w - (-11.516667)).abs() < 0.0001);
    }

    #[test]
    fn test_nmea_gga_basic() {
        let mut decoder = NmeaDecoder::new();
        // Checksum 0x4F is correct for this sentence
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        let data = feed_string(&mut decoder, sentence).unwrap();

        assert!((data.latitude - 48.1173).abs() < 0.0001);
        assert!((data.longitude - 11.516667).abs() < 0.0001);
        assert_eq!(data.satellites, 8);
        assert!((data.hdop - 0.9).abs() < 0.01);
        assert!((data.altitude - 545.4).abs() < 0.1);
        assert_eq!(data.fix, GpsFix::Fix3D);
        assert!(data.valid);
    }

    #[test]
    fn test_nmea_rmc_basic() {
        let mut decoder = NmeaDecoder::new();
        let sentence = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let data = feed_string(&mut decoder, sentence).unwrap();

        assert!((data.latitude - 48.1173).abs() < 0.0001);
        assert!((data.longitude - 11.516667).abs() < 0.0001);
        // 22.4 knots * 0.514444 = 11.52 m/s
        assert!((data.speed - 11.52).abs() < 0.1);
        assert!((data.course - 84.4).abs() < 0.1);
    }

    #[test]
    fn test_nmea_rejects_bad_checksum() {
        let mut decoder = NmeaDecoder::new();
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*FF\r\n";
        assert!(feed_string(&mut decoder, sentence).is_none());
    }

    #[test]
    fn test_nmea_recovers_from_garbage() {
        let mut decoder = NmeaDecoder::new();

        // Feed garbage
        for _ in 0..100 {
            decoder.decode(0xFF);
        }

        // Then valid sentence (checksum 0x4F)
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        assert!(feed_string(&mut decoder, sentence).is_some());
    }

    #[test]
    fn test_nmea_handles_restart_on_dollar() {
        let mut decoder = NmeaDecoder::new();

        // Partial sentence then restart (checksum 0x4F)
        let partial = "$GPGGA,123$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        assert!(feed_string(&mut decoder, partial).is_some());
    }

    #[test]
    fn test_nmea_gnss_talker_ids() {
        let mut decoder = NmeaDecoder::new();

        // GLONASS talker ID (checksum 0x53 for GLGGA: GPGGA=0x4F, P^L=0x1C, 0x4F^0x1C=0x53)
        let sentence = "$GLGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*53\r\n";
        assert!(feed_string(&mut decoder, sentence).is_some());

        // Combined GNSS talker ID (checksum 0x51 for GNGGA: GPGGA=0x4F, P^N=0x1E, 0x4F^0x1E=0x51)
        decoder.reset();
        let sentence = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*51\r\n";
        assert!(feed_string(&mut decoder, sentence).is_some());
    }

    #[test]
    fn test_nmea_invalid_rmc() {
        let mut decoder = NmeaDecoder::new();

        // RMC with void status
        let sentence = "$GPRMC,123519,V,,,,,,,230394,,,*2E\r\n";
        let data = feed_string(&mut decoder, sentence);

        // Should parse but with invalid data
        if let Some(d) = data {
            assert!(!d.valid);
        }
    }

    #[test]
    fn test_gps_fix_methods() {
        assert!(!GpsFix::NoFix.has_fix());
        assert!(GpsFix::Fix2D.has_fix());
        assert!(GpsFix::Fix3D.has_fix());
        assert!(GpsFix::DGps.has_fix());
        assert!(GpsFix::Rtk.has_fix());

        assert!(!GpsFix::NoFix.is_3d());
        assert!(!GpsFix::Fix2D.is_3d());
        assert!(GpsFix::Fix3D.is_3d());
        assert!(GpsFix::Rtk.is_3d());
    }

    // ========================================================================
    // UBX Decoder Tests
    // ========================================================================

    /// Feed bytes to UBX decoder
    fn feed_ubx(decoder: &mut UbxDecoder, data: &[u8]) -> Option<GpsData> {
        let mut result = None;
        for &byte in data {
            if let Some(gps) = decoder.decode(byte) {
                result = Some(gps);
            }
        }
        result
    }

    /// Build a minimal NAV-PVT message for testing
    fn build_test_nav_pvt(lat_deg: f64, lon_deg: f64, alt_m: f32, speed_mps: f32) -> [u8; 100] {
        let mut msg = [0u8; 100];

        // Sync bytes
        msg[0] = UBX_SYNC1;
        msg[1] = UBX_SYNC2;

        // Class + ID
        msg[2] = UBX_CLASS_NAV;
        msg[3] = UBX_NAV_PVT;

        // Length (92 bytes)
        msg[4] = 92;
        msg[5] = 0;

        // Payload starts at byte 6
        let payload = &mut msg[6..98];

        // Byte 8-10: hour:min:sec (12:35:19)
        payload[8] = 12;  // hour
        payload[9] = 35;  // min
        payload[10] = 19; // sec

        // Byte 11: valid flags (date + time valid)
        payload[11] = 0x03;

        // Byte 20: fixType (3 = 3D fix)
        payload[20] = 3;

        // Byte 21: flags (gnssFixOK)
        payload[21] = 0x01;

        // Byte 23: numSV
        payload[23] = 12;

        // Byte 24-27: lon (1e-7 degrees)
        let lon_raw = (lon_deg * 1e7) as i32;
        payload[24..28].copy_from_slice(&lon_raw.to_le_bytes());

        // Byte 28-31: lat (1e-7 degrees)
        let lat_raw = (lat_deg * 1e7) as i32;
        payload[28..32].copy_from_slice(&lat_raw.to_le_bytes());

        // Byte 36-39: hMSL (mm)
        let hmsl = (alt_m * 1000.0) as i32;
        payload[36..40].copy_from_slice(&hmsl.to_le_bytes());

        // Byte 60-63: gSpeed (mm/s)
        let gspeed = (speed_mps * 1000.0) as i32;
        payload[60..64].copy_from_slice(&gspeed.to_le_bytes());

        // Byte 64-67: headMot (1e-5 degrees) - 45.0 degrees
        let head_mot = (45.0 * 1e5) as i32;
        payload[64..68].copy_from_slice(&head_mot.to_le_bytes());

        // Byte 76-77: pDOP (0.01 scale) - 1.5
        let pdop = 150u16;
        payload[76..78].copy_from_slice(&pdop.to_le_bytes());

        // Calculate checksum over class, id, length, payload (bytes 2-97)
        let (ck_a, ck_b) = UbxConfigBuilder::checksum(&msg[2..98]);
        msg[98] = ck_a;
        msg[99] = ck_b;

        msg
    }

    #[test]
    fn test_ubx_nav_pvt_basic() {
        let mut decoder = UbxDecoder::new();

        // Build a test NAV-PVT message
        let msg = build_test_nav_pvt(48.1173, 11.516667, 545.4, 5.0);

        let data = feed_ubx(&mut decoder, &msg).unwrap();

        assert!((data.latitude - 48.1173).abs() < 0.0001);
        assert!((data.longitude - 11.516667).abs() < 0.0001);
        assert!((data.altitude - 545.4).abs() < 0.1);
        assert!((data.speed - 5.0).abs() < 0.01);
        assert!((data.course - 45.0).abs() < 0.1);
        assert_eq!(data.satellites, 12);
        assert_eq!(data.fix, GpsFix::Fix3D);
        assert!(data.valid);
    }

    #[test]
    fn test_ubx_rejects_bad_checksum() {
        let mut decoder = UbxDecoder::new();

        let mut msg = build_test_nav_pvt(48.0, 11.0, 100.0, 1.0);
        // Corrupt checksum
        msg[98] = 0xFF;
        msg[99] = 0xFF;

        assert!(feed_ubx(&mut decoder, &msg).is_none());
    }

    #[test]
    fn test_ubx_recovers_from_garbage() {
        let mut decoder = UbxDecoder::new();

        // Feed garbage
        for _ in 0..100 {
            decoder.decode(0xFF);
        }

        // Then valid message
        let msg = build_test_nav_pvt(48.0, 11.0, 100.0, 1.0);
        assert!(feed_ubx(&mut decoder, &msg).is_some());
    }

    #[test]
    fn test_ubx_cfg_rate_builder() {
        let mut buf = [0u8; 16];
        let len = UbxConfigBuilder::build_cfg_rate(100, &mut buf);

        assert_eq!(len, 14);
        assert_eq!(buf[0], UBX_SYNC1);
        assert_eq!(buf[1], UBX_SYNC2);
        assert_eq!(buf[2], UBX_CLASS_CFG);
        assert_eq!(buf[3], UBX_CFG_RATE);
        assert_eq!(buf[4], 6); // payload length
        assert_eq!(buf[6], 100); // measRate low (100ms)
        assert_eq!(buf[7], 0);   // measRate high

        // Verify checksum by feeding to decoder (should not crash)
        let (ck_a, ck_b) = UbxConfigBuilder::checksum(&buf[2..12]);
        assert_eq!(buf[12], ck_a);
        assert_eq!(buf[13], ck_b);
    }

    #[test]
    fn test_ubx_cfg_msg_builder() {
        let mut buf = [0u8; 16];
        let len = UbxConfigBuilder::build_cfg_msg(UBX_CLASS_NAV, UBX_NAV_PVT, 1, &mut buf);

        assert_eq!(len, 11);
        assert_eq!(buf[6], UBX_CLASS_NAV);
        assert_eq!(buf[7], UBX_NAV_PVT);
        assert_eq!(buf[8], 1); // rate
    }

    // ========================================================================
    // GPS Server Tests
    // ========================================================================

    #[test]
    fn test_gps_server_nmea() {
        let mut server = GpsServer::new(GpsConfig::default());

        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        let mut result = None;
        for byte in sentence.bytes() {
            if let Some(data) = server.process_byte(byte) {
                result = Some(data);
            }
        }

        let data = result.unwrap();
        assert!(server.has_fix());
        assert!((data.latitude - 48.1173).abs() < 0.0001);
    }

    #[test]
    fn test_gps_server_ubx() {
        let mut server = GpsServer::new(GpsConfig::default());

        let msg = build_test_nav_pvt(48.1173, 11.516667, 545.4, 5.0);
        let mut result = None;
        for &byte in &msg {
            if let Some(data) = server.process_byte(byte) {
                result = Some(data);
            }
        }

        let data = result.unwrap();
        assert!(server.has_fix());
        assert!((data.latitude - 48.1173).abs() < 0.0001);
    }

    #[test]
    fn test_gps_server_timeout() {
        let config = GpsConfig { timeout_ms: 1000 };
        let mut server = GpsServer::new(config);

        // Feed valid data
        let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for byte in sentence.bytes() {
            server.process_byte(byte);
        }

        assert!(server.has_fix());

        // Time passes
        server.update(500);
        assert!(server.has_fix()); // Still valid

        server.update(600);
        assert!(!server.has_fix()); // Timeout exceeded
    }

    #[test]
    fn test_gps_server_mixed_protocols() {
        let mut server = GpsServer::new(GpsConfig::default());

        // First NMEA
        let nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for byte in nmea.bytes() {
            server.process_byte(byte);
        }
        assert!((server.last_data().latitude - 48.1173).abs() < 0.0001);

        // Then UBX with different position
        let ubx = build_test_nav_pvt(52.0, 13.0, 100.0, 1.0);
        for &byte in &ubx {
            server.process_byte(byte);
        }
        assert!((server.last_data().latitude - 52.0).abs() < 0.0001);
    }
}
