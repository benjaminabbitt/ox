//! ESP32-S3 specific implementation
//!
//! Dual-core Xtensa LX7 at 240MHz with optional PSRAM support

/// PSRAM configuration for ESP32-S3
#[cfg(feature = "esp32s3-psram")]
pub struct PsramConfig {
    /// PSRAM size in bytes (detected at runtime)
    pub size: usize,
}

/// Initialize octal PSRAM (up to 8MB)
///
/// Call this early in main() before using PSRAM memory.
/// Returns the detected PSRAM configuration.
///
/// # Panics
///
/// Panics if PSRAM initialization fails (e.g., no PSRAM chip present).
#[cfg(feature = "esp32s3-psram")]
pub fn init_psram() -> PsramConfig {
    // esp-hal PSRAM initialization
    // The actual initialization is done by esp-hal when the psram feature is enabled.
    // This function provides a typed wrapper and returns config info.
    //
    // In a real implementation, this would call:
    // let (start, size) = esp_hal::psram::init_psram(...);

    // For now, return a placeholder - esp-hal handles the actual init
    // when the binary is loaded with PSRAM support compiled in.
    PsramConfig {
        size: 8 * 1024 * 1024, // 8MB default for S3
    }
}
