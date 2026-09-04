#[test]
fn read_manufacturer_id_works() {
    // Arrange
    let expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0xF9, 0x00, 0x00], vec![0x00, 0x54, 0x49]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let id = invoke!(ina229.manufacturer_id()).expect("id to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(id, 0x5449);
    spi.done();
}

#[test]
fn read_device_id_works() {
    // Arrange
    let expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0xFD, 0x00, 0x00], vec![0x00, 0x22, 0x91]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let id = invoke!(ina229.device_id()).expect("id to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(id, 0x2291);
    spi.done();
}
