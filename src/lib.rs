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
    ShuntVoltage = 0x04,
    BusVoltage = 0x05,
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

const BUS_VOLTAGE_UV_PER_LSB: f64 = 195.3125;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_0: f64 = 312.5;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_1: f64 = 78.125;

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
        let mut buffer = [
            get_frame(Register::Configuration, Command::Write),
            0x00,
            0x00,
        ];
        BigEndian::write_u16_into(&[configuration.bits()], &mut buffer[1..3]);
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.write(&buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
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
    use super::{get_frame, Command, Register};
    use byteorder::{BigEndian, ByteOrder};

    #[test]
    fn get_frame_manufacturer_read() {
        let result = get_frame(Register::ManufacturerID, Command::Read);
        assert_eq!(result, 0b1111_1001);
    }

    #[test]
    fn convert_manufacturer_response() {
        let buffer = [0x00, 0x54, 0x49];
        let value = BigEndian::read_u16(&buffer[1..3]);
        assert_eq!(value, 0x5449);
    }

    #[test]
    fn get_frame_device_read() {
        let result = get_frame(Register::DeviceID, Command::Read);
        assert_eq!(result, 0b1111_1101);
    }
}
