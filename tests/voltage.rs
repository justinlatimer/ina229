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

// BOVL (Bus Over-Voltage Threshold) register tests
#[test]
fn read_bus_overvoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x39, 0x00, 0x00],
        vec![0x00, 0x41, 0x00],
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
        .bus_overvoltage_threshold_raw()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 16640);
    spi.done();
    ncs.done();
}

#[test]
fn write_bus_overvoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x38, 0x41, 0x00])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_overvoltage_threshold_raw(16640)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_bus_overvoltage_threshold_millivolts_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x38, 0x41, 0x00])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_overvoltage_threshold_millivolts(52000.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

// BUVL (Bus Under-Voltage Threshold) register tests
#[test]
fn read_bus_undervoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::transfer(
        vec![0x3D, 0x00, 0x00],
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
        .bus_undervoltage_threshold_raw()
        .expect("reading to be returned");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert_eq!(reading, 3200);
    spi.done();
    ncs.done();
}

#[test]
fn write_bus_undervoltage_threshold_raw_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x3C, 0x0C, 0x80])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_undervoltage_threshold_raw(3200)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_bus_undervoltage_threshold_millivolts_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x3C, 0x0C, 0x80])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_undervoltage_threshold_millivolts(10000.0)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

// BOVL / BUVL: bit-15 is reserved, and returns Error::InvalidRawValue if set
#[test]
fn set_bus_overvoltage_threshold_raw_rejects_reserved_bit_works() {
    // Arrange: bit-15 set (0x8000) must be rejected before any SPI traffic occurs.
    let spi_expectations = [];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let result = ina229.set_bus_overvoltage_threshold_raw(0x8000);

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
fn set_bus_overvoltage_threshold_raw_accepts_max_valid_value_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x38, 0x7F, 0xFF])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_overvoltage_threshold_raw(0x7FFF)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_bus_overvoltage_threshold_millivolts_rejects_out_of_range_value_works() {
    // Arrange: 200 V is well past the max the 15-bit register can represent
    // (200000.0 / 3.125 = 64000, i.e. raw 0xFA00, which has bit-15 set).
    let spi_expectations = [];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let result = ina229.set_bus_overvoltage_threshold_millivolts(200000.0);

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert!(matches!(result, Err(ina229::Error::InvalidRawValue(64000))));
    spi.done();
    ncs.done();
}

#[test]
fn set_bus_undervoltage_threshold_raw_rejects_reserved_bit_works() {
    // Arrange
    let spi_expectations = [];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let result = ina229.set_bus_undervoltage_threshold_raw(0x8000);

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
fn set_bus_undervoltage_threshold_raw_accepts_max_valid_value_works() {
    // Arrange
    let spi_expectations = [SPITransaction::write(vec![0x3C, 0x7F, 0xFF])];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    ina229
        .set_bus_undervoltage_threshold_raw(0x7FFF)
        .expect("value to be written");

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    spi.done();
    ncs.done();
}

#[test]
fn set_bus_undervoltage_threshold_millivolts_rejects_out_of_range_value_works() {
    // Arrange
    let spi_expectations = [];
    let spi = SPIMock::new(&spi_expectations);
    let expectations = [];
    let ncs = PinMock::new(&expectations);
    let mut ina229 = INA229::new(spi, ncs);

    // Act
    let result = ina229.set_bus_undervoltage_threshold_millivolts(200000.0);

    // Assert
    let (mut spi, mut ncs) = ina229.release();
    assert!(matches!(result, Err(ina229::Error::InvalidRawValue(64000))));
    spi.done();
    ncs.done();
}
