#[test]
fn read_configuration_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x01, 0x00, 0x00], vec![0x00, 0x00, 0x10]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let configuration = invoke!(ina229.configuration()).expect("configuration to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(configuration, Configuration::ADCRANGE);
    spi.done();
}
