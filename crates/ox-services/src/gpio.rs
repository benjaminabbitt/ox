//! GPIO server
//!
//! Provides GPIO access to other servers/apps via IPC.
//!
//! The GPIO server manages pin access with:
//! - Pin mode configuration (input, output, pull-up, pull-down)
//! - Read/write operations
//! - Pin toggling
//! - Access control via capabilities

use ox_hal::gpio::PinMode;

/// Maximum number of pins the server can manage
pub const MAX_PINS: usize = 32;

/// GPIO command
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpioCommand {
    /// Configure pin mode
    SetMode { pin: u8, mode: PinMode },
    /// Write to output pin
    Write { pin: u8, value: bool },
    /// Read from input pin
    Read { pin: u8 },
    /// Toggle output pin
    Toggle { pin: u8 },
}

/// GPIO response
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpioResponse {
    /// Operation succeeded
    Ok,
    /// Read value
    Value(bool),
    /// Error occurred
    Error(GpioError),
}

/// GPIO error codes
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpioError {
    /// Pin number out of range
    InvalidPin,
    /// Pin not configured for this operation
    NotConfigured,
    /// No capability for this pin
    PermissionDenied,
    /// Hardware error
    HardwareError,
}

/// Pin state tracking
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PinState {
    /// Pin not configured
    Unconfigured,
    /// Configured as input
    Input,
    /// Configured as output (with current value)
    Output(bool),
}

impl Default for PinState {
    fn default() -> Self {
        PinState::Unconfigured
    }
}

/// GPIO server core logic
///
/// Manages a set of pins and handles commands.
/// For testing, we use mock pins; for real hardware, we use HAL pins.
pub struct GpioServer<const N: usize> {
    /// Pin states
    states: [PinState; N],
    /// Pin read values (for testing/simulation)
    read_values: [bool; N],
}

impl<const N: usize> GpioServer<N> {
    /// Create a new GPIO server
    pub fn new() -> Self {
        Self {
            states: [PinState::default(); N],
            read_values: [false; N],
        }
    }

    /// Handle a GPIO command
    pub fn handle_command(&mut self, cmd: GpioCommand) -> GpioResponse {
        match cmd {
            GpioCommand::SetMode { pin, mode } => self.set_mode(pin, mode),
            GpioCommand::Write { pin, value } => self.write(pin, value),
            GpioCommand::Read { pin } => self.read(pin),
            GpioCommand::Toggle { pin } => self.toggle(pin),
        }
    }

    /// Configure pin mode
    fn set_mode(&mut self, pin: u8, mode: PinMode) -> GpioResponse {
        if pin as usize >= N {
            return GpioResponse::Error(GpioError::InvalidPin);
        }

        let state = match mode {
            PinMode::Input | PinMode::InputPullUp | PinMode::InputPullDown => {
                PinState::Input
            }
            PinMode::Output => {
                PinState::Output(false)
            }
        };

        self.states[pin as usize] = state;
        GpioResponse::Ok
    }

    /// Write to output pin
    fn write(&mut self, pin: u8, value: bool) -> GpioResponse {
        if pin as usize >= N {
            return GpioResponse::Error(GpioError::InvalidPin);
        }

        match self.states[pin as usize] {
            PinState::Output(_) => {
                self.states[pin as usize] = PinState::Output(value);
                GpioResponse::Ok
            }
            PinState::Unconfigured => GpioResponse::Error(GpioError::NotConfigured),
            PinState::Input => GpioResponse::Error(GpioError::NotConfigured),
        }
    }

    /// Read from input pin
    fn read(&mut self, pin: u8) -> GpioResponse {
        if pin as usize >= N {
            return GpioResponse::Error(GpioError::InvalidPin);
        }

        match self.states[pin as usize] {
            PinState::Input => {
                GpioResponse::Value(self.read_values[pin as usize])
            }
            PinState::Output(value) => {
                // Reading from output returns current output value
                GpioResponse::Value(value)
            }
            PinState::Unconfigured => GpioResponse::Error(GpioError::NotConfigured),
        }
    }

    /// Toggle output pin
    fn toggle(&mut self, pin: u8) -> GpioResponse {
        if pin as usize >= N {
            return GpioResponse::Error(GpioError::InvalidPin);
        }

        match self.states[pin as usize] {
            PinState::Output(current) => {
                self.states[pin as usize] = PinState::Output(!current);
                GpioResponse::Ok
            }
            PinState::Unconfigured => GpioResponse::Error(GpioError::NotConfigured),
            PinState::Input => GpioResponse::Error(GpioError::NotConfigured),
        }
    }

    /// Get pin state (for testing)
    pub fn pin_state(&self, pin: u8) -> Option<PinState> {
        if pin as usize >= N {
            None
        } else {
            Some(self.states[pin as usize])
        }
    }

    /// Set simulated input value (for testing)
    pub fn set_input_value(&mut self, pin: u8, value: bool) {
        if (pin as usize) < N {
            self.read_values[pin as usize] = value;
        }
    }
}

impl<const N: usize> Default for GpioServer<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_server() -> GpioServer<8> {
        GpioServer::new()
    }

    // === Pin Configuration Tests ===

    #[test]
    fn pins_start_unconfigured() {
        let server = make_server();
        assert_eq!(server.pin_state(0), Some(PinState::Unconfigured));
        assert_eq!(server.pin_state(7), Some(PinState::Unconfigured));
    }

    #[test]
    fn set_mode_output_configures_pin() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::SetMode {
            pin: 0,
            mode: PinMode::Output,
        });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(0), Some(PinState::Output(false)));
    }

    #[test]
    fn set_mode_input_configures_pin() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::SetMode {
            pin: 1,
            mode: PinMode::Input,
        });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(1), Some(PinState::Input));
    }

    #[test]
    fn set_mode_input_pullup_configures_pin() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::SetMode {
            pin: 2,
            mode: PinMode::InputPullUp,
        });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(2), Some(PinState::Input));
    }

    #[test]
    fn set_mode_invalid_pin_returns_error() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::SetMode {
            pin: 99,
            mode: PinMode::Output,
        });
        assert_eq!(resp, GpioResponse::Error(GpioError::InvalidPin));
    }

    // === Write Tests ===

    #[test]
    fn write_high_to_output_succeeds() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });

        let resp = server.handle_command(GpioCommand::Write { pin: 0, value: true });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(0), Some(PinState::Output(true)));
    }

    #[test]
    fn write_low_to_output_succeeds() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });
        server.handle_command(GpioCommand::Write { pin: 0, value: true });

        let resp = server.handle_command(GpioCommand::Write { pin: 0, value: false });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(0), Some(PinState::Output(false)));
    }

    #[test]
    fn write_to_unconfigured_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Write { pin: 0, value: true });
        assert_eq!(resp, GpioResponse::Error(GpioError::NotConfigured));
    }

    #[test]
    fn write_to_input_pin_fails() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Input });

        let resp = server.handle_command(GpioCommand::Write { pin: 0, value: true });
        assert_eq!(resp, GpioResponse::Error(GpioError::NotConfigured));
    }

    #[test]
    fn write_to_invalid_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Write { pin: 99, value: true });
        assert_eq!(resp, GpioResponse::Error(GpioError::InvalidPin));
    }

    // === Read Tests ===

    #[test]
    fn read_from_input_returns_value() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Input });
        server.set_input_value(0, true);

        let resp = server.handle_command(GpioCommand::Read { pin: 0 });
        assert_eq!(resp, GpioResponse::Value(true));
    }

    #[test]
    fn read_from_input_returns_low_by_default() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Input });

        let resp = server.handle_command(GpioCommand::Read { pin: 0 });
        assert_eq!(resp, GpioResponse::Value(false));
    }

    #[test]
    fn read_from_output_returns_output_value() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });
        server.handle_command(GpioCommand::Write { pin: 0, value: true });

        let resp = server.handle_command(GpioCommand::Read { pin: 0 });
        assert_eq!(resp, GpioResponse::Value(true));
    }

    #[test]
    fn read_from_unconfigured_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Read { pin: 0 });
        assert_eq!(resp, GpioResponse::Error(GpioError::NotConfigured));
    }

    #[test]
    fn read_from_invalid_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Read { pin: 99 });
        assert_eq!(resp, GpioResponse::Error(GpioError::InvalidPin));
    }

    // === Toggle Tests ===

    #[test]
    fn toggle_output_from_low_to_high() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });

        let resp = server.handle_command(GpioCommand::Toggle { pin: 0 });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(0), Some(PinState::Output(true)));
    }

    #[test]
    fn toggle_output_from_high_to_low() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });
        server.handle_command(GpioCommand::Write { pin: 0, value: true });

        let resp = server.handle_command(GpioCommand::Toggle { pin: 0 });
        assert_eq!(resp, GpioResponse::Ok);
        assert_eq!(server.pin_state(0), Some(PinState::Output(false)));
    }

    #[test]
    fn toggle_unconfigured_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Toggle { pin: 0 });
        assert_eq!(resp, GpioResponse::Error(GpioError::NotConfigured));
    }

    #[test]
    fn toggle_input_pin_fails() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Input });

        let resp = server.handle_command(GpioCommand::Toggle { pin: 0 });
        assert_eq!(resp, GpioResponse::Error(GpioError::NotConfigured));
    }

    #[test]
    fn toggle_invalid_pin_fails() {
        let mut server = make_server();
        let resp = server.handle_command(GpioCommand::Toggle { pin: 99 });
        assert_eq!(resp, GpioResponse::Error(GpioError::InvalidPin));
    }

    // === Multiple Pin Tests ===

    #[test]
    fn multiple_pins_independent() {
        let mut server = make_server();
        server.handle_command(GpioCommand::SetMode { pin: 0, mode: PinMode::Output });
        server.handle_command(GpioCommand::SetMode { pin: 1, mode: PinMode::Input });
        server.handle_command(GpioCommand::Write { pin: 0, value: true });

        assert_eq!(server.pin_state(0), Some(PinState::Output(true)));
        assert_eq!(server.pin_state(1), Some(PinState::Input));
        assert_eq!(server.pin_state(2), Some(PinState::Unconfigured));
    }

    // === Edge Cases ===

    #[test]
    fn pin_state_out_of_range_returns_none() {
        let server = make_server();
        assert_eq!(server.pin_state(99), None);
    }

    #[test]
    fn default_creates_new_server() {
        let server: GpioServer<8> = GpioServer::default();
        assert_eq!(server.pin_state(0), Some(PinState::Unconfigured));
    }

    // === Type Comparisons ===

    #[test]
    fn commands_are_comparable() {
        let cmd1 = GpioCommand::Write { pin: 0, value: true };
        let cmd2 = GpioCommand::Write { pin: 0, value: true };
        let cmd3 = GpioCommand::Read { pin: 0 };

        assert_eq!(cmd1, cmd2);
        assert_ne!(cmd1, cmd3);
    }

    #[test]
    fn responses_are_comparable() {
        assert_eq!(GpioResponse::Ok, GpioResponse::Ok);
        assert_eq!(GpioResponse::Value(true), GpioResponse::Value(true));
        assert_ne!(GpioResponse::Ok, GpioResponse::Value(true));
    }

    #[test]
    fn errors_are_comparable() {
        assert_eq!(GpioError::InvalidPin, GpioError::InvalidPin);
        assert_ne!(GpioError::InvalidPin, GpioError::NotConfigured);
    }
}
