# embedded-driver

embedded-hal compatible drivers for different kinds of Sensors, Displays, or IC modules.

## Why another driver crate?

- Rust embedded driver libraries are always outdated
- Most of them are over-engineering
  - 🥲 I just want a sensor value, you've abstracted all the hidden commands in builder style
- Most of them are not compatible with newest embedded-hal

