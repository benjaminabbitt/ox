//! Chip-specific implementations
//!
//! This crate provides chip-specific initialization and configuration
//! for ESP32-C3, ESP32-C6, and ESP32-H2.

#![no_std]

#[cfg(feature = "esp32c3")]
pub mod c3;

#[cfg(feature = "esp32c6")]
pub mod c6;

#[cfg(feature = "esp32h2")]
pub mod h2;

/// Chip capabilities
pub struct ChipInfo {
    pub name: &'static str,
    pub cores: u8,
    pub has_wifi: bool,
    pub has_ble: bool,
    pub has_thread: bool,
    pub max_freq_mhz: u32,
}

#[cfg(feature = "esp32c3")]
pub const CHIP: ChipInfo = ChipInfo {
    name: "ESP32-C3",
    cores: 1,
    has_wifi: true,
    has_ble: true,
    has_thread: false,
    max_freq_mhz: 160,
};

#[cfg(feature = "esp32c6")]
pub const CHIP: ChipInfo = ChipInfo {
    name: "ESP32-C6",
    cores: 2, // HP + LP
    has_wifi: true,
    has_ble: true,
    has_thread: true,
    max_freq_mhz: 160,
};

#[cfg(feature = "esp32h2")]
pub const CHIP: ChipInfo = ChipInfo {
    name: "ESP32-H2",
    cores: 1,
    has_wifi: false,
    has_ble: true,
    has_thread: true,
    max_freq_mhz: 96,
};
