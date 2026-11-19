# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-11-20

### Added

- `INA229Async`, an async driver behind the `async` feature using
  `embedded-hal-async` SPI.
- ALERT configuration and status support via `DiagAlert`,
  `set_alert_configuration()`, and `alert_configuration()`.
- Alert threshold getters and setters for shunt voltage, bus voltage,
  temperature, and power.
- `Display` and `core::error::Error` implementations for `Error`.

### Changed

- **Breaking:** Updated from `embedded-hal` 0.2 to 1.0. `INA229::new()` now takes
  an `SpiDevice` instead of separate SPI and chip-select objects, and
  `release()` returns the SPI device.
- **Breaking:** `Error` now has a single SPI error parameter. The
  `ChipSelectError` variant was removed and `InvalidRawValue` was added.
- **Breaking:** `set_shunt_calibration()` and `calibrate()` reject values above
  `0x7FFF`.
- Updated `bitflags` from 1 to 2.
- Refactored internal register access and expanded documentation.
- Declared Rust 1.81 as the minimum supported version.

### Fixed

- Calibration values are validated before being written.
- Corrected a README typo and broken documentation links.

## [0.1.0] - 2023-01-30

- Initial release.
