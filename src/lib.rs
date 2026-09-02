//! This is a platform agnostic Rust driver for the [`INA229`], an SPI output
//! current/voltage/power monitor with alerts, using the [`embedded-hal`] traits.
//!
//! [`INA229`]: https://www.ti.com/product/INA229
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Callibrate the device. See [`callibrate()`].
//! - Read the shunt voltage. See [`shunt_voltage_nanovolts()`].
//! - Read the bus voltage. See [`bus_voltage_microvolts()`].
//! - Read the current. See [`current_amps()`].
//! - Read the power. See [`power_watts()`].
//!
//! [`callibrate()`]: struct.INA229.html#method.callibrate
//! [`shunt_voltage_nanovolts()`]: struct.INA229.html#method.shunt_voltage_nanovolts
//! [`bus_voltage_microvolts()`]: struct.INA229.html#method.bus_voltage_microvolts
//! [`current_amps()`]: struct.INA229.html#method.current_amps
//! [`power_watts()`]: struct.INA229.html#method.power_watts
//!
//! ## The device
//!
//! The INA229-Q1 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC
//! specifically designed for current-sensing applications. The device can measure a full-scale
//! differential input of ±163.84 mV or ±40.96 mV across a resistive shunt sense element with
//! common-mode voltage support from –0.3 V to +85 V.
//!
//! Datasheet:
//! - [INA229](https://www.ti.com/lit/gpn/ina229)

#![warn(unsafe_code, missing_docs)]
#![no_std]

pub mod ina229;
pub use ina229::{Configuration, DiagAlert, Error, INA229, MODE};
