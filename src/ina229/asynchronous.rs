use super::*;
use embedded_hal_async::spi::SpiDevice;

impl<SPI, SPIError> INA229Async<SPI>
where
    SPI: SpiDevice<u8, Error = SPIError>,
{
    async fn read_register_u16(&mut self, register: Register) -> Result<u16, Error<SPIError>> {
        let mut buffer = read_register_buffer::<3>(register);
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SPIError)?;
        Ok(read_u16(&buffer[1..3]))
    }

    async fn write_register_u16(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<SPIError>> {
        let buffer = write_register_u16_buffer(register, value);
        self.spi.write(&buffer).await.map_err(Error::SPIError)?;
        Ok(())
    }

    async fn read_register_i16(&mut self, register: Register) -> Result<i16, Error<SPIError>> {
        let mut buffer = read_register_buffer::<3>(register);
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SPIError)?;
        Ok(read_i16(&buffer[1..3]))
    }

    async fn write_register_i16(
        &mut self,
        register: Register,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_u16(register, value as u16).await
    }

    async fn read_register_u24(&mut self, register: Register) -> Result<u32, Error<SPIError>> {
        let mut buffer = read_register_buffer::<4>(register);
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SPIError)?;
        Ok(read_u24(&buffer[1..4]))
    }

    async fn read_register_i24(&mut self, register: Register) -> Result<i32, Error<SPIError>> {
        let mut buffer = read_register_buffer::<4>(register);
        self.spi
            .transfer_in_place(&mut buffer)
            .await
            .map_err(Error::SPIError)?;
        Ok(read_i24(&buffer[1..4]))
    }

    /// Sets the CONFIG register with the value provided.
    pub async fn set_configuration(
        &mut self,
        configuration: Configuration,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_u16(Register::Configuration, configuration.bits())
            .await?;
        self.state.set_configuration(configuration);
        Ok(())
    }

    /// Get the configuration.
    pub async fn configuration(&mut self) -> Result<Configuration, Error<SPIError>> {
        self.read_register_u16(Register::Configuration)
            .await
            .map(Configuration::from_bits_truncate)
            .inspect(|&config| {
                self.state.set_configuration(config);
            })
    }

    /// Configure ALERT pin behaviour by writing to the DIAG_ALERT register.
    pub async fn set_alert_configuration(
        &mut self,
        configuration: DiagAlert,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_u16(Register::DiagAlert, alert_configuration_raw(configuration))
            .await
    }

    /// Read the DIAG_ALERT register, giving the current ALERT pin configuration.
    /// Note: if [`DiagAlert::ALATCH`] is enabled, reading this register clears the ALERT pin.
    pub async fn alert_configuration(&mut self) -> Result<DiagAlert, Error<SPIError>> {
        self.read_register_u16(Register::DiagAlert)
            .await
            .map(DiagAlert::from_bits_truncate)
    }

    /// Gets the value from the shunt calibration register.
    pub async fn shunt_calibration(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::ShuntCalibration).await
    }

    /// Sets the shunt calibration register to the value provided.
    /// Note: bit-15 of SHUNT_CAL is reserved, so we require value no larger than 0x7FFF.
    pub async fn set_shunt_calibration(&mut self, value: u16) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, SHUNT_CALIBRATION_MAX_RAW)?;
        self.write_register_u16(Register::ShuntCalibration, value)
            .await
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    /// Note: oversized inputs are rejected with [`Error::InvalidRawValue`]. Because the
    /// computed value is cast to `u16` before validation, values beyond `u16::MAX` are
    /// reported as `0xFFFF`.
    pub async fn calibrate(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError>> {
        if let Some((current_lsb, value)) = self
            .state
            .calibration_value(shunt_resistance, current_expected_max)
        {
            self.set_shunt_calibration(value).await?;
            self.state.set_current_lsb(current_lsb);
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    pub async fn configure_and_calibrate(
        &mut self,
        configuration: Configuration,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_configuration(configuration).await?;
        self.calibrate(shunt_resistance, current_expected_max).await
    }

    /// Get the raw bus voltage reading.
    pub async fn bus_voltage_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register_i24(Register::BusVoltage)
            .await
            .map(measurement_raw_from_register)
    }

    /// Get the bus voltage reading in microvolts.
    pub async fn bus_voltage_microvolts(&mut self) -> Result<f64, Error<SPIError>> {
        self.bus_voltage_raw()
            .await
            .map(bus_voltage_microvolts_from_raw)
    }

    /// Get the raw shunt voltage reading.
    pub async fn shunt_voltage_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register_i24(Register::ShuntVoltage)
            .await
            .map(measurement_raw_from_register)
    }

    /// Get the shunt voltage reading in nanovolts.
    pub async fn shunt_voltage_nanovolts(&mut self) -> Result<f64, Error<SPIError>> {
        let configuration = self.state.configuration().ok_or(Error::NotConfigured)?;
        self.shunt_voltage_raw()
            .await
            .map(|value| shunt_voltage_nanovolts_from_raw(configuration, value))
    }

    /// Get the raw die temperature value.
    pub async fn temperature_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register_i16(Register::DieTemperature).await
    }

    /// Get the die temperature in millidegrees Celsius.
    pub async fn temperature_millidegrees_celsius(&mut self) -> Result<f64, Error<SPIError>> {
        self.temperature_raw()
            .await
            .map(temperature_millidegrees_celsius_from_raw)
    }

    /// Get the raw value from the current register.
    pub async fn current_raw(&mut self) -> Result<i32, Error<SPIError>> {
        self.read_register_i24(Register::Current)
            .await
            .map(measurement_raw_from_register)
    }

    /// Get the current reading in Amps.
    pub async fn current_amps(&mut self) -> Result<f64, Error<SPIError>> {
        let current_lsb = self.state.current_lsb().ok_or(Error::NotConfigured)?;
        self.current_raw()
            .await
            .map(|value| current_amps_from_raw(current_lsb, value))
    }

    /// Get the raw value from the power register
    pub async fn power_raw(&mut self) -> Result<u32, Error<SPIError>> {
        self.read_register_u24(Register::Power).await
    }

    /// Get the power reading in Watts.
    pub async fn power_watts(&mut self) -> Result<f64, Error<SPIError>> {
        let current_lsb = self.state.current_lsb().ok_or(Error::NotConfigured)?;
        self.power_raw()
            .await
            .map(|value| power_watts_from_raw(current_lsb, value))
    }

    /// Get the raw shunt overvoltage threshold, from the SOVL register.
    pub async fn shunt_overvoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register_i16(Register::SOVL).await
    }

    /// Set the raw shunt overvoltage threshold, in the SOVL register.
    pub async fn set_shunt_overvoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_i16(Register::SOVL, value).await
    }

    /// Set the shunt overvoltage threshold in microvolts. Requires the INA229 to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub async fn set_shunt_overvoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .shunt_threshold_raw(microvolts)
            .ok_or(Error::NotConfigured)?;
        self.set_shunt_overvoltage_threshold_raw(value).await
    }

    /// Get the raw shunt undervoltage threshold, from the SUVL register.
    pub async fn shunt_undervoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register_i16(Register::SUVL).await
    }

    /// Set the raw shunt undervoltage threshold, in the SUVL register.
    pub async fn set_shunt_undervoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_i16(Register::SUVL, value).await
    }

    /// Set the shunt undervoltage threshold in microvolts. Requires the device to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub async fn set_shunt_undervoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .shunt_threshold_raw(microvolts)
            .ok_or(Error::NotConfigured)?;
        self.set_shunt_undervoltage_threshold_raw(value).await
    }

    /// Get the raw bus overvoltage threshold, from the BOVL register.
    pub async fn bus_overvoltage_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::BOVL).await
    }

    /// Set the raw bus overvoltage threshold, in the BOVL register.
    /// Note: bit-15 of BOVL is reserved, so we require value no larger than 0x7FFF.
    pub async fn set_bus_overvoltage_threshold_raw(
        &mut self,
        value: u16,
    ) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, BUS_THRESHOLD_MAX_RAW)?;
        self.write_register_u16(Register::BOVL, value).await
    }

    /// Set the bus overvoltage threshold in millivolts.
    pub async fn set_bus_overvoltage_threshold_millivolts(
        &mut self,
        millivolts: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_bus_overvoltage_threshold_raw(bus_threshold_raw(millivolts))
            .await
    }

    /// Get the raw bus undervoltage threshold, from the BUVL register.
    pub async fn bus_undervoltage_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::BUVL).await
    }

    /// Set the raw bus undervoltage threshold, in the BUVL register.
    /// Note: bit-15 of BUVL is reserved, so we require value no larger than 0x7FFF.
    pub async fn set_bus_undervoltage_threshold_raw(
        &mut self,
        value: u16,
    ) -> Result<(), Error<SPIError>> {
        validate_raw_value(value, BUS_THRESHOLD_MAX_RAW)?;
        self.write_register_u16(Register::BUVL, value).await
    }

    /// Set the bus undervoltage threshold in millivolts.
    pub async fn set_bus_undervoltage_threshold_millivolts(
        &mut self,
        millivolts: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_bus_undervoltage_threshold_raw(bus_threshold_raw(millivolts))
            .await
    }

    /// Get the raw temperature over-limit threshold, from the TEMP_LIMIT register.
    pub async fn temperature_overlimit_threshold_raw(&mut self) -> Result<i16, Error<SPIError>> {
        self.read_register_i16(Register::TOLT).await
    }

    /// Set the raw temperature over-limit threshold, in the TEMP_LIMIT register.
    pub async fn set_temperature_overlimit_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_i16(Register::TOLT, value).await
    }

    /// Set the temperature over-limit threshold in millidegrees Celsius.
    pub async fn set_temperature_overlimit_threshold_millidegrees_celsius(
        &mut self,
        millidegrees_celsius: f64,
    ) -> Result<(), Error<SPIError>> {
        self.set_temperature_overlimit_threshold_raw(temperature_threshold_raw(
            millidegrees_celsius,
        ))
        .await
    }

    /// Get the raw power over-limit threshold, from the PWR_LIMIT register.
    pub async fn power_overlimit_threshold_raw(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::POLT).await
    }

    /// Set the raw power over-limit threshold, in the PWR_LIMIT register.
    pub async fn set_power_overlimit_threshold_raw(
        &mut self,
        value: u16,
    ) -> Result<(), Error<SPIError>> {
        self.write_register_u16(Register::POLT, value).await
    }

    /// Set the power over-limit threshold in watts. Requires device to be calibrated.
    pub async fn set_power_overlimit_threshold_watts(
        &mut self,
        watts: f64,
    ) -> Result<(), Error<SPIError>> {
        let value = self
            .state
            .power_threshold_raw(watts)
            .ok_or(Error::NotConfigured)?;
        self.set_power_overlimit_threshold_raw(value).await
    }

    /// Get the unique manufacturer identification number.
    pub async fn manufacturer_id(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::ManufacturerID).await
    }

    /// Get the unique die identification number.
    pub async fn device_id(&mut self) -> Result<u16, Error<SPIError>> {
        self.read_register_u16(Register::DeviceID).await
    }
}
