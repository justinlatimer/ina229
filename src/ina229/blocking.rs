use super::registers::*;
use super::*;
use embedded_hal::spi::SpiDevice;

/// Blocking INA229 voltage/current/power monitor.
///
/// # Example
///
/// ```no_run
/// # fn example<SPI>(spi: SPI) -> Result<(), ina229::Error<SPI::Error>>
/// # where
/// #     SPI: embedded_hal::spi::SpiDevice<u8>,
/// # {
/// let mut monitor = ina229::INA229::new(spi);
/// let voltage = monitor.bus_voltage_microvolts()?;
/// # let _ = voltage;
/// # Ok(())
/// # }
/// ```
pub struct INA229<SPI> {
    spi: SPI,
    state: State,
}

impl<SPI> INA229<SPI> {
    /// Create a new instance of an INA229 device.
    pub fn new(spi: SPI) -> Self {
        INA229 {
            spi,
            state: State::default(),
        }
    }

    /// Destroy the INA229 instance and return the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI, SPIError> INA229<SPI>
where
    SPI: SpiDevice<u8, Error = SPIError>,
{
    fn read_register<Register: ReadableRegister>(
        &mut self,
    ) -> Result<Register::Value, Error<SPIError>> {
        let mut buffer = read_register_buffer::<Register>();
        self.spi
            .transfer_in_place(&mut buffer[..Register::FRAME_LEN])
            .map_err(Error::SPIError)?;
        Ok(Register::decode(&buffer[1..Register::FRAME_LEN]))
    }

    fn write_register<Register: WritableRegister>(
        &mut self,
        value: Register::Value,
    ) -> Result<(), Error<SPIError>> {
        let buffer = write_register_buffer::<Register>(value);
        self.spi
            .write(&buffer[..Register::FRAME_LEN])
            .map_err(Error::SPIError)?;
        Ok(())
    }

    /// Sets the CONFIG register with the value provided.
    pub fn set_configuration(
        &mut self,
        configuration: Configuration,
    ) -> Result<(), Error<SPIError>> {
        self.write_register::<ConfigurationRegister>(configuration.bits())?;
        self.state.set_configuration(configuration);
        Ok(())
    }

    /// Get the configuration.
    pub fn configuration(&mut self) -> Result<Configuration, Error<SPIError>> {
        self.read_register::<ConfigurationRegister>()
            .map(Configuration::from_bits_truncate)
            .inspect(|&config| {
                self.state.set_configuration(config);
            })
    }

    /// Configure ALERT pin behaviour by writing to the DIAG_ALERT register.
    pub fn set_alert_configuration(
        &mut self,
        configuration: DiagAlert,
    ) -> Result<(), Error<SPIError>> {
        self.write_register::<DiagAlertRegister>(alert_configuration_raw(configuration))
    }

    /// Read the DIAG_ALERT register, giving the current ALERT pin configuration.
    /// Note: if [`DiagAlert::ALATCH`] is enabled, reading this register clears the ALERT pin.
    pub fn alert_configuration(&mut self) -> Result<DiagAlert, Error<SPIError>> {
        self.read_register::<DiagAlertRegister>()
            .map(DiagAlert::from_bits_truncate)
    }

    /// Gets the value from the shunt calibration register.
    pub fn shunt_calibration(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<ShuntCalibrationRegister>()
    }

    /// Sets the shunt calibration register to the value provided.
    /// Note: bit-15 of SHUNT_CAL is reserved, so we require value no larger than 0x7FFF.
    pub fn set_shunt_calibration(&mut self, value: u16) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, SHUNT_CALIBRATION_MAX_RAW)?;
        self.write_register::<ShuntCalibrationRegister>(value)
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    /// Note: oversized inputs are rejected with [`Error::InvalidRawValue`]. Because the
    /// computed value is cast to `u16` before validation, values beyond `u16::MAX` are
    /// reported as `0xFFFF`.
    pub fn calibrate(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError>> {
        if let Some((current_lsb, value)) = self
            .state
            .calibration_value(shunt_resistance, current_expected_max)
        {
            self.set_shunt_calibration(value)?;
            self.state.set_current_lsb(current_lsb);
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    pub fn configure_and_calibrate(
        &mut self,
        configuration: Configuration,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_configuration(configuration)
            .and_then(|_| self.calibrate(shunt_resistance, current_expected_max))
    }

    /// Get the raw bus voltage reading.
    pub fn bus_voltage_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register::<BusVoltageRegister>()
            .map(|value| value as i32)
    }

    /// Get the bus voltage reading in microvolts.
    pub fn bus_voltage_microvolts(&mut self) -> Result<f64, Error<SPIError>> {
        self.bus_voltage_raw().map(bus_voltage_microvolts_from_raw)
    }

    /// Get the raw shunt voltage reading.
    pub fn shunt_voltage_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register::<ShuntVoltageRegister>()
    }

    /// Get the shunt voltage reading in nanovolts.
    pub fn shunt_voltage_nanovolts(&mut self) -> Result<f64, Error<SPIError>> {
        let configuration = self.state.configuration().ok_or(Error::NotConfigured)?;
        self.shunt_voltage_raw()
            .map(|value| shunt_voltage_nanovolts_from_raw(configuration, value))
    }

    /// Get the raw die temperature value.
    pub fn temperature_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register::<DieTemperatureRegister>()
    }

    /// Get the die temperature in millidegrees Celsius.
    pub fn temperature_millidegrees_celsius(&mut self) -> Result<f64, Error<SPIError>> {
        self.temperature_raw()
            .map(temperature_millidegrees_celsius_from_raw)
    }

    /// Get the raw value from the current register.
    pub fn current_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register::<CurrentRegister>()
    }

    /// Get the current reading in Amps.
    pub fn current_amps(&mut self) -> Result<f64, Error<SPIError>> {
        let current_lsb = self.state.current_lsb().ok_or(Error::NotConfigured)?;
        self.current_raw()
            .map(|value| current_amps_from_raw(current_lsb, value))
    }

    /// Get the raw value from the power register
    pub fn power_raw(&mut self) -> Result<u32, Error<SPIError>> {
        self.read_register::<PowerRegister>()
    }

    /// Get the power reading in Watts.
    pub fn power_watts(&mut self) -> Result<f64, Error<SPIError>> {
        let current_lsb = self.state.current_lsb().ok_or(Error::NotConfigured)?;
        self.power_raw()
            .map(|value| power_watts_from_raw(current_lsb, value))
    }

    /// Get the raw shunt overvoltage threshold, from the SOVL register.
    pub fn shunt_overvoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register::<ShuntOverVoltageRegister>()
    }

    /// Set the raw shunt overvoltage threshold, in the SOVL register.
    pub fn set_shunt_overvoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register::<ShuntOverVoltageRegister>(value)
    }

    /// Set the shunt overvoltage threshold in microvolts. Requires the INA229 to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub fn set_shunt_overvoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .shunt_threshold_raw(microvolts)
            .ok_or(Error::NotConfigured)?;
        self.set_shunt_overvoltage_threshold_raw(value)
    }

    /// Get the raw shunt undervoltage threshold, from the SUVL register.
    pub fn shunt_undervoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register::<ShuntUnderVoltageRegister>()
    }

    /// Set the raw shunt undervoltage threshold, in the SUVL register.
    pub fn set_shunt_undervoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register::<ShuntUnderVoltageRegister>(value)
    }

    /// Set the shunt undervoltage threshold in microvolts. Requires the device to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub fn set_shunt_undervoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .shunt_threshold_raw(microvolts)
            .ok_or(Error::NotConfigured)?;
        self.set_shunt_undervoltage_threshold_raw(value)
    }

    /// Get the raw bus overvoltage threshold, from the BOVL register.
    pub fn bus_overvoltage_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<BusOverVoltageRegister>()
    }

    /// Set the raw bus overvoltage threshold, in the BOVL register.
    /// Note: bit-15 of BOVL is reserved, so we require value no larger than 0x7FFF.
    pub fn set_bus_overvoltage_threshold_raw(&mut self, value: u16) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, BUS_THRESHOLD_MAX_RAW)?;
        self.write_register::<BusOverVoltageRegister>(value)
    }

    /// Set the bus overvoltage threshold in millivolts.
    pub fn set_bus_overvoltage_threshold_millivolts(
        &mut self,
        millivolts: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_bus_overvoltage_threshold_raw(bus_threshold_raw(millivolts))
    }

    /// Get the raw bus undervoltage threshold, from the BUVL register.
    pub fn bus_undervoltage_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<BusUnderVoltageRegister>()
    }

    /// Set the raw bus undervoltage threshold, in the BUVL register.
    /// Note: bit-15 of BUVL is reserved, so we require value no larger than 0x7FFF.
    pub fn set_bus_undervoltage_threshold_raw(
        &mut self,
        value: u16,
    ) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, BUS_THRESHOLD_MAX_RAW)?;
        self.write_register::<BusUnderVoltageRegister>(value)
    }

    /// Set the bus undervoltage threshold in millivolts.
    pub fn set_bus_undervoltage_threshold_millivolts(
        &mut self,
        millivolts: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_bus_undervoltage_threshold_raw(bus_threshold_raw(millivolts))
    }

    /// Get the raw temperature over-limit threshold, from the TEMP_LIMIT register.
    pub fn temperature_overlimit_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register::<TemperatureOverLimitRegister>()
    }

    /// Set the raw temperature over-limit threshold, in the TEMP_LIMIT register.
    pub fn set_temperature_overlimit_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register::<TemperatureOverLimitRegister>(value)
    }

    /// Set the temperature over-limit threshold in millidegrees Celsius.
    pub fn set_temperature_overlimit_threshold_millidegrees_celsius(
        &mut self,
        millidegrees_celsius: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_temperature_overlimit_threshold_raw(temperature_threshold_raw(
            millidegrees_celsius,
        ))
    }

    /// Get the raw power over-limit threshold, from the PWR_LIMIT register.
    pub fn power_overlimit_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<PowerOverLimitRegister>()
    }

    /// Set the raw power over-limit threshold, in the PWR_LIMIT register.
    pub fn set_power_overlimit_threshold_raw(&mut self, value: u16) -> Result<(), Error<SPIError>> {
        self.write_register::<PowerOverLimitRegister>(value)
    }

    /// Set the power over-limit threshold in watts. Requires device to be calibrated.
    pub fn set_power_overlimit_threshold_watts(
        &mut self,
        watts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .power_threshold_raw(watts)
            .ok_or(Error::NotConfigured)?;
        self.set_power_overlimit_threshold_raw(value)
    }

    /// Get the unique manufacturer identification number.
    pub fn manufacturer_id(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<ManufacturerIdRegister>()
    }

    /// Get the unique die identification number.
    pub fn device_id(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register::<DeviceIdRegister>()
    }
}
