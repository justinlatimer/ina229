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

// SHUNT_CAL: bit-15 is reserved, and returns Error::InvalidRawValue if set
#[test]
fn set_shunt_calibration_rejects_reserved_bit() {
    // Arrange: bit-15 set (0x8000) must be rejected before any SPI traffic occurs.
    let spi_expectations = [];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let result = ina229.set_shunt_calibration(0x8000);

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert!(matches!(
        result,
        Err(ina229::Error::InvalidRawValue(0x8000))
    ));
    spi.done();
    ncs.done();
}

#[test]
fn set_shunt_calibration_accepts_max_valid_value() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x08, 0x7F, 0xFF])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_shunt_calibration(0x7FFF)
        .expect("value to be written");

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

#[test]
fn calibrate_rejects_shunt_calibration_value_over_max() {
    // Arrange: these inputs compute a shunt calibration far above 0x7FFF, which
    // must be rejected before any SHUNT_CAL write occurs. Only the CONFIG write
    // is expected on the bus.
    let spi_expectations = [SPITransaction::write(vec![0x00, 0x00, 0x00])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("config to be set");

    // Act
    let result = ina229.calibrate(1.0, 100000.0);

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert!(matches!(
        result,
        Err(ina229::Error::InvalidRawValue(0xFFFF))
    ));
    spi.done();
    ncs.done();
}
