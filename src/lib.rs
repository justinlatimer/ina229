#![no_std]

use core::convert::Infallible;

use embedded_hal::{
    blocking::spi,
    digital::v2::OutputPin,
    spi::{Mode, MODE_1},
};

pub const MODE: Mode = MODE_1;

pub struct INA229<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
}

impl<SPI, NCS, E> INA229<SPI, NCS>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin<Error = Infallible>,
{
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        INA229 { spi, ncs }
    }

    pub fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }
}
