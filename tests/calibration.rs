use embedded_hal_mock::pin::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::{Configuration, INA229};

#[test]
fn read_shunt_calibration_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x09, 0x00, 0x00],
        vec![0x00, 0x10, 0x00],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let reading = ina229.shunt_calibration().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 4096);
    spi.done();
    ncs.done();
}

#[test]
fn write_shunt_calibration_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x08, 0x10, 0x00])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_shunt_calibration(4096)
        .expect("write to succeed");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn calibrate_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::write(vec![0x08, 0x0F, 0xD2]),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("config to be set");

    // Act
    ina229
        .calibrate(0.0162, 10.0)
        .expect("calibration to succeed");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}
