---
sidebar_position: 1
---

# Installation

Get Ox running on your machine.

## Prerequisites

### Rust Toolchain

Install Rust via rustup:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Ox requires nightly Rust for embedded features:

```bash
rustup default nightly
rustup component add rust-src llvm-tools
```

### ESP32 Tools

Install the ESP32 flash utility:

```bash
cargo install espflash
```

For ESP32-S3 (Xtensa architecture), install the ESP toolchain:

```bash
cargo install espup
espup install
```

After installation, source the environment:

```bash
source ~/export-esp.sh  # Add to your shell profile
```

### Just (Optional)

Ox uses [just](https://github.com/casey/just) as a command runner:

```bash
cargo install just
```

## Clone and Build

```bash
git clone https://github.com/example/ox
cd ox

# Build for ESP32-C3 (default)
just build

# Or without just:
cargo build --release -p ox-app
```

## Verify Installation

Connect an ESP32-C3 via USB and flash:

```bash
just run
```

You should see boot messages in the serial monitor.

## Supported Chips

| Chip | Target | Command |
|------|--------|---------|
| ESP32-C3 | `riscv32imc-unknown-none-elf` | `just build` |
| ESP32-C6 | `riscv32imc-unknown-none-elf` | `just build-c6` |
| ESP32-H2 | `riscv32imc-unknown-none-elf` | `just build-h2` |
| ESP32-S3 | `xtensa-esp32s3-none-elf` | `just build-s3` |

## IDE Setup

### VS Code

Install the rust-analyzer extension. Add to `.vscode/settings.json`:

```json
{
    "rust-analyzer.cargo.target": "riscv32imc-unknown-none-elf",
    "rust-analyzer.checkOnSave.allTargets": false
}
```

### CLion / RustRover

Use the Rust plugin. Set the cargo target in Settings → Languages & Frameworks → Rust.

## Troubleshooting

### "can't find crate for std"

Ox is `no_std`. Ensure you're building for an embedded target:

```bash
cargo build --target riscv32imc-unknown-none-elf
```

### "espflash: No serial ports found"

- Check USB cable (some are charge-only)
- Install USB drivers for your OS
- Check permissions: `sudo usermod -aG dialout $USER` (Linux)

### "xtensa target not found"

Run `espup install` and source the environment:

```bash
source ~/export-esp.sh
```
