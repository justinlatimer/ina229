use approx::assert_relative_eq;
use embedded_hal_mock::pin::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::{Configuration, INA229};

#[test]
fn read_current_raw_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::transfer(vec![0x1D, 0x00, 0x00, 0x00], vec![0x00, 0x4C, 0xCC, 0xC0]),
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
        .expect("configuration to be set");

    // Act
    let reading = ina229.current_raw().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 314572);
    spi.done();
    ncs.done();
}

#[test]
fn read_current_amps_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::write(vec![0x08, 0x0F, 0xD2]),
        SPITransaction::transfer(vec![0x1D, 0x00, 0x00, 0x00], vec![0x00, 0x4C, 0xCC, 0xC0]),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("configuration to be set");
    ina229
        .calibrate(0.0162, 10.0)
        .expect("calibration to be set");

    // Act
    let reading = ina229.current_amps().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_relative_eq!(reading, 6.0, max_relative = 0.00001);
    spi.done();
    ncs.done();
}
