use embedded_hal_mock::pin::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::INA229;

#[test]
fn read_manufacturer_id_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0xF9, 0x00, 0x00],
        vec![0x00, 0x54, 0x49],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let id = ina229.manufacturer_id().expect("id to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(id, 0x5449);
    spi.done();
    ncs.done();
}

#[test]
fn read_device_id_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0xFD, 0x00, 0x00],
        vec![0x00, 0x22, 0x91],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let id = ina229.device_id().expect("id to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(id, 0x2291);
    spi.done();
    ncs.done();
}
