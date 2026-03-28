# Ox Robotics Microkernel - Build Targets

# Default target
default: build

# ============== Setup ==============

# Install development tools (wokwi-cli, espflash)
setup:
    ./scripts/setup-tools.sh

# Install Xtensa toolchain for ESP32-S3 (requires espup)
setup-xtensa:
    @echo "Installing espup (ESP Rust toolchain manager)..."
    cargo install espup
    @echo "Installing Xtensa Rust toolchain..."
    espup install
    @echo ""
    @echo "Done! To use the Xtensa toolchain, run:"
    @echo "  source $HOME/export-esp.sh"
    @echo "Then build with: just build-s3"

# ============== Build Commands ==============

# Build for ESP32-C3 (default)
build:
    cargo build --release -p ox-app

# Build debug version
build-debug:
    cargo build -p ox-app

# Build for ESP32-C6 with dual-core
build-c6:
    cargo build --release -p ox-app --no-default-features --features chip-esp32c6

# Build for ESP32-H2
build-h2:
    cargo build --release -p ox-app --no-default-features --features chip-esp32h2

# Build for ESP32-S3 (Xtensa - requires: just setup-xtensa && source ~/export-esp.sh)
build-s3:
    cargo build --release -p ox-app --no-default-features --features chip-esp32s3 --target xtensa-esp32s3-none-elf

# Build for ESP32-S3 with PSRAM
build-s3-psram:
    cargo build --release -p ox-app --no-default-features --features chip-esp32s3-psram --target xtensa-esp32s3-none-elf

# Build all chip variants (RISC-V only - S3 requires Xtensa toolchain)
build-all: build build-c6 build-h2

# Build all including S3 (requires Xtensa toolchain)
build-all-xtensa: build build-c6 build-h2 build-s3

# ============== Flash & Run ==============

# Flash and monitor (ESP32-C3)
run: build
    espflash flash --monitor target/riscv32imc-unknown-none-elf/release/ox-app

# Flash debug build
run-debug: build-debug
    espflash flash --monitor target/riscv32imc-unknown-none-elf/debug/ox-app

# Flash ESP32-C6
run-c6: build-c6
    espflash flash --monitor target/riscv32imc-unknown-none-elf/release/ox-app

# Flash ESP32-S3 (Xtensa)
run-s3: build-s3
    espflash flash --monitor target/xtensa-esp32s3-none-elf/release/ox-app

# Just monitor (already flashed)
monitor:
    espflash monitor

# ============== Testing ==============

# Run host-side tests (on native target, not embedded)
test:
    cargo test --lib -p ox-control --target x86_64-unknown-linux-gnu
    cargo test --lib -p ox-hal --target x86_64-unknown-linux-gnu
    cargo test --lib -p ox-kernel --target x86_64-unknown-linux-gnu

# Run all tests with verbose output
test-verbose:
    cargo test --lib -p ox-control --target x86_64-unknown-linux-gnu -- --nocapture
    cargo test --lib -p ox-hal --target x86_64-unknown-linux-gnu -- --nocapture
    cargo test --lib -p ox-kernel --target x86_64-unknown-linux-gnu -- --nocapture

# Run property-based tests (ox-control PID etc.)
test-props:
    cargo test --lib -p ox-control --target x86_64-unknown-linux-gnu -- --nocapture

# ============== Development ==============

# Check compilation without building
check:
    cargo check --workspace

# Run clippy lints
lint:
    cargo clippy --workspace -- -D warnings

# Format code
fmt:
    cargo fmt --all

# Format check
fmt-check:
    cargo fmt --all -- --check

# Clean build artifacts
clean:
    cargo clean

# ============== Simulation (Wokwi) ==============
# Note: Requires WOKWI_CLI_TOKEN environment variable
# Get your free token at: https://wokwi.com/dashboard/ci

# Run in Wokwi simulator (ESP32-C3) - 30 second timeout
sim: build
    @echo "Running Wokwi simulation..."
    @echo "Ensure WOKWI_CLI_TOKEN is set (get it at https://wokwi.com/dashboard/ci)"
    wokwi-cli --timeout 30000 .

# Run in Wokwi simulator indefinitely (Ctrl+C to stop)
sim-interactive: build
    wokwi-cli .

# Run simulation with GDB server (connect with: riscv32-unknown-elf-gdb -ex "target remote :3333")
sim-debug: build
    wokwi-cli --gdb --gdb-port 3333 .

# ============== Debug ==============

# Debug with probe-rs
debug: build-debug
    probe-rs debug --chip esp32c3 target/riscv32imc-unknown-none-elf/debug/ox-app

# Run with defmt output
defmt: build
    probe-rs run --chip esp32c3 target/riscv32imc-unknown-none-elf/release/ox-app
