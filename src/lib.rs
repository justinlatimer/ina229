//! This is a platform agnostic Rust driver for the [`INA229`], an SPI output
//! current/voltage/power monitor with alerts, using the [`embedded-hal`] traits.
//!
//! [`INA229`]: https://www.ti.com/product/INA229
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Calibrate the device. See [`calibrate()`].
//! - Read the shunt voltage. See [`shunt_voltage_nanovolts()`].
//! - Read the bus voltage. See [`bus_voltage_microvolts()`].
//! - Read the current. See [`current_amps()`].
//! - Read the power. See [`power_watts()`].
//! - Read the die temperature. See [`temperature_millidegrees_celsius()`].
//! - Configure the ALERT pin and read the alert status. See [`set_alert_configuration()`].
//! - Set alert thresholds for shunt voltage, bus voltage, temperature and power:
//!   - Shunt overvoltage: [`set_shunt_overvoltage_threshold_microvolts()`]
//!   - Shunt undervoltage: [`set_shunt_undervoltage_threshold_microvolts()`]
//!   - Bus overvoltage: [`set_bus_overvoltage_threshold_millivolts()`]
//!   - Bus undervoltage: [`set_bus_undervoltage_threshold_millivolts()`]
//!   - Temperature over-limit: [`set_temperature_overlimit_threshold_millidegrees_celsius()`]
//!   - Power over-limit: [`set_power_overlimit_threshold_watts()`]
//! - Read the manufacturer and device ID. See [`manufacturer_id()`].
//!
//! [`calibrate()`]: INA229::calibrate
//! [`shunt_voltage_nanovolts()`]: INA229::shunt_voltage_nanovolts
//! [`bus_voltage_microvolts()`]: INA229::bus_voltage_microvolts
//! [`current_amps()`]: INA229::current_amps
//! [`power_watts()`]: INA229::power_watts
//! [`temperature_millidegrees_celsius()`]: INA229::temperature_millidegrees_celsius
//! [`set_alert_configuration()`]: INA229::set_alert_configuration
//! [`set_shunt_overvoltage_threshold_microvolts()`]: INA229::set_shunt_overvoltage_threshold_microvolts
//! [`set_shunt_undervoltage_threshold_microvolts()`]: INA229::set_shunt_undervoltage_threshold_microvolts
//! [`set_bus_overvoltage_threshold_millivolts()`]: INA229::set_bus_overvoltage_threshold_millivolts
//! [`set_bus_undervoltage_threshold_millivolts()`]: INA229::set_bus_undervoltage_threshold_millivolts
//! [`set_temperature_overlimit_threshold_millidegrees_celsius()`]: INA229::set_temperature_overlimit_threshold_millidegrees_celsius
//! [`set_power_overlimit_threshold_watts()`]: INA229::set_power_overlimit_threshold_watts
//! [`manufacturer_id()`]: INA229::manufacturer_id
//!
//! ## The device
//!
//! The INA229 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC
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
