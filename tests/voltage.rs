use embedded_hal_mock::pin::{Mock as PinMock, State as PinState, Transaction as PinTransaction};
use embedded_hal_mock::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::{Configuration, INA229};

#[test]
fn read_bus_voltage_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x15, 0x00, 0x00, 0x00],
        vec![0x00, 0x3C, 0x00, 0x00],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let reading = ina229.bus_voltage_raw().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 245760);
    spi.done();
    ncs.done();
}

#[test]
fn read_bus_voltage_microvolts_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x15, 0x00, 0x00, 0x00],
        vec![0x00, 0x3C, 0x00, 0x00],
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
        .bus_voltage_microvolts()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 48000000.0);
    spi.done();
    ncs.done();
}

#[test]
fn read_shunt_voltage_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x11, 0x00, 0x00, 0x00],
        vec![0x00, 0x4B, 0xF0, 0x00],
    )];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let reading = ina229.shunt_voltage_raw().expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 311040);
    spi.done();
    ncs.done();
}

#[test]
fn read_shunt_voltage_nanovolts_adcmode_0_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::transfer(vec![0x11, 0x00, 0x00, 0x00], vec![0x00, 0x4B, 0xF0, 0x00]),
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
    let reading = ina229
        .shunt_voltage_nanovolts()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 97200000.0);
    spi.done();
    ncs.done();
}

#[test]
fn read_shunt_voltage_nanovolts_adcmode_1_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x10]),
        SPITransaction::transfer(vec![0x11, 0x00, 0x00, 0x00], vec![0x00, 0x4B, 0xF0, 0x00]),
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
        .set_configuration(Configuration::ADCRANGE)
        .expect("config to be set");

    // Act
    let reading = ina229
        .shunt_voltage_nanovolts()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 24300000.0);
    spi.done();
    ncs.done();
}

// SOVL (Shunt Over-Voltage Threshold) register tests
#[test]
fn read_shunt_overvoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x31, 0x00, 0x00],
        vec![0x00, 0x7E, 0x90],
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
        .shunt_overvoltage_threshold_raw()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 32400);
    spi.done();
    ncs.done();
}

#[test]
fn write_shunt_overvoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x30, 0x7E, 0x90])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_shunt_overvoltage_threshold_raw(32400)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_shunt_overvoltage_threshold_microvolts_adcmode_0_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::write(vec![0x30, 0x7E, 0x90]),
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
        .set_shunt_overvoltage_threshold_microvolts(162000.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_shunt_overvoltage_threshold_microvolts_adcmode_1_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x10]),
        SPITransaction::write(vec![0x30, 0x7E, 0x90]),
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
        .set_configuration(Configuration::ADCRANGE)
        .expect("config to be set");

    // Act
    ina229
        .set_shunt_overvoltage_threshold_microvolts(40500.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

// SUVL (Shunt Under-Voltage Threshold) register tests
#[test]
fn read_shunt_undervoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x35, 0x00, 0x00],
        vec![0x00, 0x81, 0x70],
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
        .shunt_undervoltage_threshold_raw()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, -32400);
    spi.done();
    ncs.done();
}

#[test]
fn write_shunt_undervoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x34, 0x81, 0x70])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_shunt_undervoltage_threshold_raw(-32400)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_shunt_undervoltage_threshold_microvolts_adcmode_0_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x00]),
        SPITransaction::write(vec![0x34, 0x81, 0x70]),
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
        .set_shunt_undervoltage_threshold_microvolts(-162000.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_shunt_undervoltage_threshold_microvolts_adcmode_1_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::write(vec![0x00, 0x00, 0x10]),
        SPITransaction::write(vec![0x34, 0x81, 0x70]),
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
        .set_configuration(Configuration::ADCRANGE)
        .expect("config to be set");

    // Act
    ina229
        .set_shunt_undervoltage_threshold_microvolts(-40500.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}
