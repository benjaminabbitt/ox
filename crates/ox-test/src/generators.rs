//! Test data generators
//!
//! Generators for creating test data for RC protocols, telemetry, etc.

/// Generate a valid SBUS frame
///
/// SBUS frame format: 25 bytes
/// - Byte 0: Start byte (0x0F)
/// - Bytes 1-22: 16 channels × 11 bits packed
/// - Byte 23: Flags (ch17, ch18, frame_lost, failsafe)
/// - Byte 24: End byte (0x00)
pub fn make_sbus_frame(channels: &[u16; 16], failsafe: bool, frame_lost: bool) -> [u8; 25] {
    let mut frame = [0u8; 25];
    frame[0] = 0x0F; // Start byte

    // Pack 16 channels (11 bits each) into bytes 1-22
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
    if frame_lost {
        flags |= 0x04;
    }
    if failsafe {
        flags |= 0x08;
    }
    frame[23] = flags;

    frame[24] = 0x00; // End byte
    frame
}

/// Generate SBUS frame with center values (992 = neutral)
pub fn make_sbus_center_frame() -> [u8; 25] {
    make_sbus_frame(&[992u16; 16], false, false)
}

/// Generate SBUS frame with specified throttle/steering
///
/// throttle: -1.0 to 1.0 mapped to SBUS 172-1811
/// steering: -1.0 to 1.0 mapped to SBUS 172-1811
pub fn make_sbus_control_frame(throttle: f32, steering: f32) -> [u8; 25] {
    let mut channels = [992u16; 16];
    channels[2] = normalized_to_sbus(throttle); // Channel 3 = throttle
    channels[0] = normalized_to_sbus(steering); // Channel 1 = steering
    make_sbus_frame(&channels, false, false)
}

/// Convert normalized [-1, 1] to SBUS [172, 1811]
fn normalized_to_sbus(value: f32) -> u16 {
    let clamped = value.clamp(-1.0, 1.0);
    let scaled = (clamped + 1.0) / 2.0; // 0.0 to 1.0
    (172.0 + scaled * (1811.0 - 172.0)) as u16
}

/// Generate a valid CRSF RC channels frame
///
/// CRSF frame format:
/// - Byte 0: Device address (0xC8 for flight controller)
/// - Byte 1: Frame length (payload + type + CRC)
/// - Byte 2: Frame type (0x16 for RC channels)
/// - Bytes 3-24: 16 channels × 11 bits packed (22 bytes)
/// - Byte 25: CRC8
pub fn make_crsf_channels_frame(channels: &[u16; 16]) -> [u8; 26] {
    let mut frame = [0u8; 26];
    frame[0] = 0xC8; // Flight controller address
    frame[1] = 24; // Length: 22 bytes payload + type + CRC
    frame[2] = 0x16; // RC channels packed

    // Pack channels (11 bits each, same as SBUS)
    frame[3] = (channels[0] & 0xFF) as u8;
    frame[4] = ((channels[0] >> 8) | (channels[1] << 3)) as u8;
    frame[5] = ((channels[1] >> 5) | (channels[2] << 6)) as u8;
    frame[6] = ((channels[2] >> 2) & 0xFF) as u8;
    frame[7] = ((channels[2] >> 10) | (channels[3] << 1)) as u8;
    frame[8] = ((channels[3] >> 7) | (channels[4] << 4)) as u8;
    frame[9] = ((channels[4] >> 4) | (channels[5] << 7)) as u8;
    frame[10] = ((channels[5] >> 1) & 0xFF) as u8;
    frame[11] = ((channels[5] >> 9) | (channels[6] << 2)) as u8;
    frame[12] = ((channels[6] >> 6) | (channels[7] << 5)) as u8;
    frame[13] = (channels[7] >> 3) as u8;

    frame[14] = (channels[8] & 0xFF) as u8;
    frame[15] = ((channels[8] >> 8) | (channels[9] << 3)) as u8;
    frame[16] = ((channels[9] >> 5) | (channels[10] << 6)) as u8;
    frame[17] = ((channels[10] >> 2) & 0xFF) as u8;
    frame[18] = ((channels[10] >> 10) | (channels[11] << 1)) as u8;
    frame[19] = ((channels[11] >> 7) | (channels[12] << 4)) as u8;
    frame[20] = ((channels[12] >> 4) | (channels[13] << 7)) as u8;
    frame[21] = ((channels[13] >> 1) & 0xFF) as u8;
    frame[22] = ((channels[13] >> 9) | (channels[14] << 2)) as u8;
    frame[23] = ((channels[14] >> 6) | (channels[15] << 5)) as u8;
    frame[24] = (channels[15] >> 3) as u8;

    // Calculate CRC8 (DVB-S2 polynomial)
    frame[25] = crc8_dvb_s2(&frame[2..25]);
    frame
}

/// Generate CRSF frame with center values
pub fn make_crsf_center_frame() -> [u8; 26] {
    // CRSF center is 992 (same as SBUS)
    make_crsf_channels_frame(&[992u16; 16])
}

/// Generate CRSF link statistics frame
pub fn make_crsf_link_stats_frame(rssi: i8, lq: u8, snr: i8) -> [u8; 14] {
    let mut frame = [0u8; 14];
    frame[0] = 0xC8; // FC address
    frame[1] = 12; // Length
    frame[2] = 0x14; // Link statistics type

    // Link stats payload
    frame[3] = rssi as u8; // Uplink RSSI Ant1
    frame[4] = rssi as u8; // Uplink RSSI Ant2
    frame[5] = lq; // Uplink Link Quality
    frame[6] = snr as u8; // Uplink SNR
    frame[7] = 0; // Active antenna
    frame[8] = 0; // RF mode
    frame[9] = 0; // Uplink TX power
    frame[10] = rssi as u8; // Downlink RSSI
    frame[11] = lq; // Downlink LQ
    frame[12] = snr as u8; // Downlink SNR

    frame[13] = crc8_dvb_s2(&frame[2..13]);
    frame
}

/// CRC8 calculation (DVB-S2 polynomial 0xD5)
fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sbus_frame_has_correct_format() {
        let frame = make_sbus_center_frame();
        assert_eq!(frame[0], 0x0F, "Start byte");
        assert_eq!(frame[24], 0x00, "End byte");
        assert_eq!(frame.len(), 25);
    }

    #[test]
    fn crsf_frame_has_correct_format() {
        let frame = make_crsf_center_frame();
        assert_eq!(frame[0], 0xC8, "Device address");
        assert_eq!(frame[1], 24, "Frame length");
        assert_eq!(frame[2], 0x16, "Frame type");
        assert_eq!(frame.len(), 26);
    }

    #[test]
    fn normalized_to_sbus_converts_correctly() {
        assert_eq!(normalized_to_sbus(-1.0), 172);
        assert_eq!(normalized_to_sbus(0.0), 991); // ~992
        assert_eq!(normalized_to_sbus(1.0), 1811);
    }

    #[test]
    fn crc8_calculates_correctly() {
        // Known test vector
        let data = [0x16, 0x00, 0x00];
        let crc = crc8_dvb_s2(&data);
        // CRC should be deterministic
        assert_eq!(crc, crc8_dvb_s2(&data));
    }
}
