#!/bin/bash
# Ox Tooling Setup Script
# Installs development tools for ESP32 RISC-V development

set -e

INSTALL_DIR="${HOME}/.local/bin"
mkdir -p "$INSTALL_DIR"

echo "=== Ox Development Tools Setup ==="
echo ""

# Check for espflash
if command -v espflash &> /dev/null; then
    echo "[OK] espflash already installed: $(espflash --version)"
else
    echo "[INSTALL] Installing espflash..."
    curl -L "https://github.com/esp-rs/espflash/releases/download/v4.3.0/espflash-x86_64-unknown-linux-gnu.zip" -o /tmp/espflash.zip
    unzip -o /tmp/espflash.zip -d "${INSTALL_DIR}"
    chmod +x "${INSTALL_DIR}/espflash"
    rm /tmp/espflash.zip
    echo "[OK] espflash installed"
fi

# Check for cargo-espflash
if command -v cargo-espflash &> /dev/null; then
    echo "[OK] cargo-espflash already installed: $(cargo-espflash --version)"
else
    echo "[INSTALL] Installing cargo-espflash..."
    curl -L "https://github.com/esp-rs/espflash/releases/download/v4.3.0/cargo-espflash-x86_64-unknown-linux-gnu.zip" -o /tmp/cargo-espflash.zip
    unzip -o /tmp/cargo-espflash.zip -d "${INSTALL_DIR}"
    chmod +x "${INSTALL_DIR}/cargo-espflash"
    rm /tmp/cargo-espflash.zip
    echo "[OK] cargo-espflash installed"
fi

# Check for probe-rs
if command -v probe-rs &> /dev/null; then
    echo "[OK] probe-rs already installed: $(probe-rs --version)"
else
    echo "[INSTALL] Installing probe-rs..."
    cargo install probe-rs-tools
    echo "[OK] probe-rs installed"
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Make sure ${INSTALL_DIR} is in your PATH:"
echo "  export PATH=\"\${HOME}/.local/bin:\${PATH}\""
echo ""
echo "Usage:"
echo "  just run      # Build, flash, and monitor"
echo "  just monitor  # Monitor serial output"
echo "  just debug    # Debug with probe-rs"
