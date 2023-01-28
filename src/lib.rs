#![no_std]

use core::convert::Infallible;
use core::result::Result;

use byteorder::{BigEndian, ByteOrder};
use embedded_hal::{
    blocking::spi::Transfer,
    digital::v2::OutputPin,
    spi::{Mode, MODE_1},
};

pub const MODE: Mode = MODE_1;

#[repr(u8)]
enum Register {
    ManufacturerID = 0x3E,
    DeviceID = 0x3F,
}

enum Command {
    Read,
    Write,
}

pub struct INA229<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
}

impl<SPI, NCS, E> INA229<SPI, NCS>
where
    SPI: Transfer<u8, Error = E>,
    NCS: OutputPin<Error = Infallible>,
{
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        INA229 { spi, ncs }
    }

    pub fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    /// Get the unique manufacturer identification number
    pub fn manufacturer_id(&mut self) -> Result<u16, E> {
        infallible(self.ncs.set_low());
        let mut buffer = [
            get_frame(Register::ManufacturerID, Command::Read),
            0x00,
            0x00,
        ];
        self.spi.transfer(&mut buffer)?;
        let value = BigEndian::read_u16(&buffer[1..3]);
        infallible(self.ncs.set_high());
        Ok(value)
    }

    /// Get the unique die identification number.
    pub fn device_id(&mut self) -> Result<u16, E> {
        infallible(self.ncs.set_low());
        let mut buffer = [get_frame(Register::DeviceID, Command::Read), 0x00, 0x00];
        self.spi.transfer(&mut buffer)?;
        let value = BigEndian::read_u16(&buffer[1..3]);
        infallible(self.ncs.set_high());
        Ok(value)
    }
}

fn get_frame(register: Register, command: Command) -> u8 {
    let frame = (register as u8) << 2u8;
    match command {
        Command::Write => frame & !0b00000001,
        Command::Read => frame | 0b00000001,
    }
}

fn infallible<T>(r: Result<T, Infallible>) -> T {
    match r {
        Ok(x) => x,
        Err(never) => match never {},
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
