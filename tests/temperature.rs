use embedded_hal_mock::pin::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::INA229;

#[test]
fn read_temperature_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x19, 0x00, 0x00],
        vec![0x00, 0x0C, 0x80],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let reading = ina229.temperature_raw().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 3200);
    spi.done();
    ncs.done();
}

#[test]
fn read_temperature_millidegrees_celsius_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x19, 0x00, 0x00],
        vec![0x00, 0x0C, 0x80],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let reading = ina229
        .temperature_millidegrees_celsius()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 25000.0);
    spi.done();
    ncs.done();
}
