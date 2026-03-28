#!/bin/bash
# Ox Tooling Setup Script
# Installs development tools for ESP32 RISC-V development

set -e

INSTALL_DIR="${HOME}/.local/bin"
mkdir -p "$INSTALL_DIR"

echo "=== Ox Development Tools Setup ==="
echo ""

# Check for wokwi-cli
if command -v wokwi-cli &> /dev/null; then
    echo "[OK] wokwi-cli already installed: $(wokwi-cli --version)"
else
    echo "[INSTALL] Installing wokwi-cli..."
    curl -L "https://github.com/wokwi/wokwi-cli/releases/download/v0.26.1/wokwi-cli-linuxstatic-x64" -o "${INSTALL_DIR}/wokwi-cli"
    chmod +x "${INSTALL_DIR}/wokwi-cli"
    echo "[OK] wokwi-cli installed"
fi

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

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Make sure ${INSTALL_DIR} is in your PATH:"
echo "  export PATH=\"\${HOME}/.local/bin:\${PATH}\""
echo ""
echo "For Wokwi simulation, get a free token at:"
echo "  https://wokwi.com/dashboard/ci"
echo ""
echo "Then set it:"
echo "  export WOKWI_CLI_TOKEN='your-token-here'"
echo ""
echo "Run 'just sim' to start the simulation!"
