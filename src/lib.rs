#![no_std]

use core::result::Result;

use bitflags::bitflags;
use byteorder::{BigEndian, ByteOrder};
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    spi::{Mode, MODE_1},
};

bitflags! {
    /// Configuration register contents.
    #[repr(C)]
    pub struct Configuration: u16 {
        /// Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset.
        /// Resets all registers to default values.
        /// 0h = Normal Operation
        /// 1h = System Reset sets registers to default values
        /// This bit self-clears.
        /// Default: 0.
        const RST       = 0b1000_0000_0000_0000;

        /// Resets the contents of accumulation registers ENERGY and CHARGE to 0.
        /// 0h = Normal Operation
        /// 1h = Clears registers to default values for ENERGY and CHARGE registers
        /// Default: 0.
        const RSTACC    = 0b0100_0000_0000_0000;

        /// Sets the Delay for initial ADC conversion in steps of 2 ms.
        /// 0h = 0 s
        /// 1h = 2 ms
        /// FFh = 510 ms
        /// Default: 0.
        const CONVDLY   = 0b0011_1111_1100_0000;

        /// Enables temperature compensation of an external shunt
        /// 0h = Shunt Temperature Compensation Disabled
        /// 1h = Shunt Temperature Compensation Enabled
        /// Default: 0.
        const TEMPCOMP  = 0b0000_0000_0010_0000;

        /// Shunt full scale range selection across IN+ and IN–.
        /// 0h = ±163.84 mV
        /// 1h = ± 40.96 mV
        /// Default: 0.
        const ADCRANGE  = 0b0000_0000_0001_0000;

        /// Reserved. Always reads 0.
        const RESERVED  = 0b0000_0000_0000_1111;
    }
}

pub const MODE: Mode = MODE_1;

#[repr(u8)]
enum Register {
    Configuration = 0x00,
    ShuntCalibration = 0x02,
    ShuntVoltage = 0x04,
    BusVoltage = 0x05,
    DieTemperature = 0x06,
    ManufacturerID = 0x3E,
    DeviceID = 0x3F,
}

enum Command {
    Read,
    Write,
}

#[derive(Debug)]
pub enum Error<SPIError, CSError> {
    NotConfigured,
    SPIError(SPIError),
    ChipSelectError(CSError),
}

// Conversion constants
const BUS_VOLTAGE_UV_PER_LSB: f64 = 195.3125;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_0: f64 = 312.5;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_1: f64 = 78.125;
const TEMPERATURE_MC_PER_LSB: f64 = 7.8125;

// Calibration constants
const DENOMINATOR: f64 = (1 << 19) as f64; // From Datasheet, 2^19
const INTERNAL_SCALING: f64 = 13107200000.0; // From Datasheet, 13107.2 * 10^6

#[inline(always)]
fn calculate_calibration_value(
    configuration: Configuration,
    shunt_resistance: f64,
    current_expected_max: f64,
) -> u16 {
    let scale = if configuration.contains(Configuration::ADCRANGE) {
        4.0
    } else {
        1.0
    };
    let current_lsb = calculate_current_lsb(current_expected_max);
    let shunt_cal = INTERNAL_SCALING * current_lsb * shunt_resistance * scale;
    shunt_cal as u16
}

#[inline(always)]
fn calculate_current_lsb(current_expected_max: f64) -> f64 {
    current_expected_max / DENOMINATOR
}

pub struct INA229<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
    config: Option<Configuration>,
}

impl<SPI, NCS, SPIError, CSError> INA229<SPI, NCS>
where
    SPI: Transfer<u8, Error = SPIError> + Write<u8, Error = SPIError>,
    NCS: OutputPin<Error = CSError>,
{
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        INA229 {
            spi,
            ncs,
            config: None,
        }
    }

    pub fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    fn read_register_u16(&mut self, register: Register) -> Result<u16, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_u16(&buffer[1..3]);
        Ok(value)
    }

    fn write_register_u16(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Write), 0x00, 0x00];
        BigEndian::write_u16_into(&[value], &mut buffer[1..3]);
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.write(&buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        Ok(())
    }

    fn read_register_i16(&mut self, register: Register) -> Result<i16, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_i16(&buffer[1..3]);
        Ok(value)
    }

    fn read_register_i24(&mut self, register: Register) -> Result<i32, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_i24(&buffer[1..4]);
        Ok(value)
    }

    /// Sets the CONFIG register with the value provided.
    pub fn set_configuration(
        &mut self,
        configuration: Configuration,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_u16(Register::Configuration, configuration.bits())?;
        self.config = Some(configuration);
        Ok(())
    }

    /// Get the configuration.
    pub fn configuration(&mut self) -> Result<Configuration, Error<SPIError, CSError>> {
        self.read_register_u16(Register::Configuration)
            .map(Configuration::from_bits_truncate)
            .map(|config| {
                self.config = Some(config);
                config
            })
    }

    /// Gets the value from the shunt calibration register.
    pub fn shunt_calibration(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::ShuntCalibration)
    }

    /// Sets the shunt calibration register to the value provided.
    pub fn set_shunt_calibration(&mut self, value: u16) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_u16(Register::ShuntCalibration, value)
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    pub fn calibrate(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        if let Some(config) = self.config {
            let value = calculate_calibration_value(config, shunt_resistance, current_expected_max);
            self.set_shunt_calibration(value)?;
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get the raw bus voltage reading.
    pub fn bus_voltage_raw(&mut self) -> Result<i32, Error<SPIError, CSError>> {
        self.read_register_i24(Register::BusVoltage).map(|x| x >> 4)
    }

    /// Get the bus voltage reading in microvolts.
    pub fn bus_voltage_microvolts(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        self.bus_voltage_raw()
            .map(|x| (x as f64) * BUS_VOLTAGE_UV_PER_LSB)
    }

    /// Get the raw shunt voltage reading.
    pub fn shunt_voltage_raw(&mut self) -> Result<i32, Error<SPIError, CSError>> {
        self.read_register_i24(Register::ShuntVoltage)
            .map(|x| x >> 4)
    }

    /// Get the shunt voltage reading in nanovolts.
    pub fn shunt_voltage_nanovolts(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        if let Some(config) = self.config {
            self.shunt_voltage_raw().map(|value| {
                if config.contains(Configuration::ADCRANGE) {
                    (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_1
                } else {
                    (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_0
                }
            })
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get the raw die temperature value.
    pub fn temperature_raw(&mut self) -> Result<i16, Error<SPIError, CSError>> {
        self.read_register_i16(Register::DieTemperature)
    }

    /// Get the die temperature in millidegrees Celsius.
    pub fn temperature_millidegrees_celsius(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        self.temperature_raw()
            .map(|x| (x as f64) * TEMPERATURE_MC_PER_LSB)
    }

    /// Get the unique manufacturer identification number.
    pub fn manufacturer_id(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::ManufacturerID)
    }

    /// Get the unique die identification number.
    pub fn device_id(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::DeviceID)
    }
}

fn get_frame(register: Register, command: Command) -> u8 {
    let frame = (register as u8) << 2u8;
    match command {
        Command::Write => frame & !0b00000001,
        Command::Read => frame | 0b00000001,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        calculate_calibration_value, calculate_current_lsb, get_frame, Command, Configuration,
        Register,
    };
    use approx::assert_relative_eq;

    #[test]
    fn get_frame_manufacturer_read() {
        let result = get_frame(Register::ManufacturerID, Command::Read);
        assert_eq!(result, 0b1111_1001);
    }

    #[test]
    fn get_frame_device_read() {
        let result = get_frame(Register::DeviceID, Command::Read);
        assert_eq!(result, 0b1111_1101);
    }

    #[test]
    fn calculate_current_lsb_works() {
        let lsb = calculate_current_lsb(10.0); // 10 Amps
        assert_relative_eq!(lsb, 0.0000190735, max_relative = 0.00001);
    }

    #[test]
    fn calculate_calibration_value_works() {
        let value = calculate_calibration_value(Configuration::from_bits_truncate(0), 0.0162, 10.0);
        assert_eq!(value, 4050);
    }
}
