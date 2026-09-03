use approx::assert_relative_eq;
use embedded_hal_mock::eh1::spi::{Mock as SPIMock, Transaction as SPITransaction};
use ina229::{Configuration, INA229};

#[test]
fn read_current_raw_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(
            vec![0x1D, 0x00, 0x00, 0x00],
            vec![0x00, 0x4C, 0xCC, 0xC0],
        ),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);

    // Act
    let reading = ina229.current_raw().expect("reading to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(reading, 314572);
    spi.done();
}

#[test]
fn read_current_amps_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x00, 0x00, 0x00]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x08, 0x0F, 0xD2]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(
            vec![0x1D, 0x00, 0x00, 0x00],
            vec![0x00, 0x4C, 0xCC, 0xC0],
        ),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("configuration to be set");
    ina229
        .calibrate(0.0162, 10.0)
        .expect("calibration to be set");

    // Act
    let reading = ina229.current_amps().expect("reading to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_relative_eq!(reading, 6.0, max_relative = 0.00001);
    spi.done();
}

#[test]
fn read_power_raw_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(
            vec![0x21, 0x00, 0x00, 0x00],
            vec![0x00, 0x48, 0x00, 0x0C],
        ),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);

    // Act
    let reading = ina229.power_raw().expect("reading to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(reading, 4718604);
    spi.done();
}

#[test]
fn read_power_watts_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x00, 0x00, 0x00]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x08, 0x0F, 0xD2]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(
            vec![0x21, 0x00, 0x00, 0x00],
            vec![0x00, 0x48, 0x00, 0x0C],
        ),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("configuration to be set");
    ina229
        .calibrate(0.0162, 10.0)
        .expect("calibration to be set");

    // Act
    let reading = ina229.power_watts().expect("reading to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_relative_eq!(reading, 288.0, max_relative = 0.00001);
    spi.done();
}

// POLT (Power Over-Limit Threshold) register tests
#[test]
fn read_power_overlimit_threshold_raw_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x45, 0x00, 0x00], vec![0x00, 0x19, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);

    // Act
    let reading = ina229
        .power_overlimit_threshold_raw()
        .expect("reading to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(reading, 6400);
    spi.done();
}

#[test]
fn write_power_overlimit_threshold_raw_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x44, 0x19, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);

    // Act
    ina229
        .set_power_overlimit_threshold_raw(6400)
        .expect("value to be written");

    // Assert
    let mut spi = ina229.release();
    spi.done();
}

#[test]
fn set_power_overlimit_threshold_watts_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x00, 0x00, 0x00]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x08, 0x0F, 0xD2]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x44, 0x19, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = INA229::new(spi);
    ina229
        .set_configuration(Configuration::from_bits_truncate(0))
        .expect("configuration to be set");
    ina229
        .calibrate(0.0162, 10.0)
        .expect("calibration to be set");

    // Act
    ina229
        .set_power_overlimit_threshold_watts(100.0)
        .expect("value to be written");

    // Assert
    let mut spi = ina229.release();
    spi.done();
}
