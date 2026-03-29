---
sidebar_position: 1
---

# Architecture Decision Records

This section documents the key technical decisions made in designing Ox. Each ADR follows a standard format:

- **Status** — Proposed, Accepted, Deprecated, or Superseded
- **Context** — Why was this decision needed?
- **Decision** — What did we decide?
- **Consequences** — What are the trade-offs?

## Why Document Decisions?

Robotics projects often fail not from bad code, but from forgotten context. Six months from now, you'll wonder "why did we do it this way?" ADRs capture the reasoning when it's fresh.

## Decision Index

| ADR | Title | Status |
|-----|-------|--------|
| [0001](/docs/decisions/0001-rust-over-python) | Use Rust Instead of Python | Accepted |
| [0002](/docs/decisions/0002-esp32-chip-selection) | Use ESP32 Over Arduino/STM32 | Accepted |
| [0003](/docs/decisions/0003-microkernel-architecture) | Microkernel Over Monolithic | Accepted |
| [0004](/docs/decisions/0004-real-time-requirements) | Real-Time Design Constraints | Accepted |
| [0005](/docs/decisions/0005-embassy-async) | Embassy Over RTOS | Accepted |

## Contributing New ADRs

When making a significant architectural decision:

1. Create a new file: `docs/decisions/NNNN-short-title.md`
2. Follow the template structure
3. Get feedback before marking as Accepted
4. ADRs are immutable once accepted — create a new ADR to supersede
