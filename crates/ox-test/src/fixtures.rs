//! Test fixtures

use ox_hal::mock::{MockMotor, MockEncoder};

/// Create a mock motor pair for testing
pub fn mock_motors() -> (MockMotor, MockMotor) {
    (MockMotor::new(), MockMotor::new())
}

/// Create a mock encoder pair for testing
pub fn mock_encoders() -> (MockEncoder, MockEncoder) {
    (MockEncoder::new(1000), MockEncoder::new(1000))
}
