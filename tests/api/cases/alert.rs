#[test]
fn read_alert_configuration_default_reset_value_works() {
    // Arrange: DIAG_ALRT resets to 0001h (MEMSTAT set, everything else clear).
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x2D, 0x00, 0x00], vec![0x00, 0x00, 0x01]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let flags = invoke!(ina229.alert_configuration()).expect("flags to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(flags, DiagAlert::MEMSTAT);
    assert!(flags.contains(DiagAlert::MEMSTAT));
    assert!(!flags.contains(DiagAlert::ALATCH));
    spi.done();
}

#[test]
fn read_alert_configuration_parses_diagnostic_status_flags_works() {
    // Arrange: SHNTOL (0x40) | BUSUL (0x08) | CNVRF (0x02) | MEMSTAT (0x01) = 0x4B.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x2D, 0x00, 0x00], vec![0x00, 0x00, 0x4B]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let flags = invoke!(ina229.alert_configuration()).expect("flags to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(
        flags,
        DiagAlert::SHNTOL | DiagAlert::BUSUL | DiagAlert::CNVRF | DiagAlert::MEMSTAT
    );
    assert!(flags.contains(DiagAlert::SHNTOL));
    assert!(flags.contains(DiagAlert::BUSUL));
    assert!(flags.contains(DiagAlert::CNVRF));
    assert!(flags.contains(DiagAlert::MEMSTAT));
    // Flags that were not set in the raw value should not be reported as set.
    assert!(!flags.contains(DiagAlert::SHNTUL));
    assert!(!flags.contains(DiagAlert::BUSOL));
    assert!(!flags.contains(DiagAlert::TMPOL));
    assert!(!flags.contains(DiagAlert::POL));
    spi.done();
}

#[test]
fn read_alert_configuration_parses_pin_config_and_overflow_flags_works() {
    // Arrange: ALATCH (0x8000) | CNVR (0x4000) | ENERGYOF (0x0800) | MATHOF (0x0200)
    // | TMPOL (0x0080) = 0xCA80.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x2D, 0x00, 0x00], vec![0x00, 0xCA, 0x80]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    let flags = invoke!(ina229.alert_configuration()).expect("flags to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(
        flags,
        DiagAlert::ALATCH
            | DiagAlert::CNVR
            | DiagAlert::ENERGYOF
            | DiagAlert::MATHOF
            | DiagAlert::TMPOL
    );
    assert!(flags.contains(DiagAlert::ALATCH));
    assert!(flags.contains(DiagAlert::CNVR));
    assert!(flags.contains(DiagAlert::ENERGYOF));
    assert!(flags.contains(DiagAlert::MATHOF));
    assert!(flags.contains(DiagAlert::TMPOL));
    // SLOWALERT and APOL were not set in the raw value.
    assert!(!flags.contains(DiagAlert::SLOWALERT));
    assert!(!flags.contains(DiagAlert::APOL));
    assert!(!flags.contains(DiagAlert::CHARGEOF));
    spi.done();
}

#[test]
fn set_alert_configuration_writes_single_bit_works() {
    // Arrange: APOL alone = 0x1000.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x2C, 0x10, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    invoke!(ina229.set_alert_configuration(DiagAlert::APOL))
        .expect("configuration to be written");

    // Assert
    let mut spi = ina229.release();
    spi.done();
}

#[test]
fn set_alert_configuration_writes_all_four_configurable_bits_works() {
    // Arrange: ALATCH | CNVR | SLOWALERT | APOL = 0xF000.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x2C, 0xF0, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    invoke!(ina229.set_alert_configuration(
        DiagAlert::ALATCH | DiagAlert::CNVR | DiagAlert::SLOWALERT | DiagAlert::APOL,
    ))
    .expect("configuration to be written");

    // Assert
    let mut spi = ina229.release();
    spi.done();
}

#[test]
fn set_alert_configuration_clears_all_configurable_bits_works() {
    // Arrange: writing an empty configuration should clear ALATCH/CNVR/SLOWALERT/APOL.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x2C, 0x00, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act
    invoke!(ina229.set_alert_configuration(DiagAlert::empty()))
        .expect("configuration to be written");

    // Assert
    let mut spi = ina229.release();
    spi.done();
}

#[test]
fn set_alert_configuration_masks_out_read_only_status_bits_works() {
    // Arrange: SHNTOL and MATHOF are read-only status flags, not configuration bits.
    // If they leaked into the write, the test would return [0x12, 0x40] instead of [0x10, 0x00]
    // (SHNTOL = 0x40 in the low byte, MATHOF = 0x02 in the high byte) and the test will fail.
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x2C, 0x10, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act: request APOL (writeable) alongside two read-only diagnostic flags.
    invoke!(ina229.set_alert_configuration(
        DiagAlert::APOL | DiagAlert::SHNTOL | DiagAlert::MATHOF,
    ))
    .expect("configuration to be written");

    // Assert: only APOL is written
    let mut spi = ina229.release();
    spi.done();
}

#[test]
fn toggle_apol_then_confirm_via_alert_configuration_works() {
    // Arrange
    let spi_expectations = [
        SPITransaction::transaction_start(),
        SPITransaction::write_vec(vec![0x2C, 0x10, 0x00]),
        SPITransaction::transaction_end(),
        SPITransaction::transaction_start(),
        SPITransaction::transfer_in_place(vec![0x2D, 0x00, 0x00], vec![0x00, 0x10, 0x00]),
        SPITransaction::transaction_end(),
    ];
    let spi = SPIMock::new(&spi_expectations);
    let mut ina229 = Driver::new(spi);

    // Act: invert the ALERT pin polarity with APOL, then read the register back to confirm
    invoke!(ina229.set_alert_configuration(DiagAlert::APOL))
        .expect("configuration to be written");
    let flags = invoke!(ina229.alert_configuration()).expect("flags to be returned");

    // Assert
    let mut spi = ina229.release();
    assert_eq!(flags, DiagAlert::APOL);
    assert!(flags.contains(DiagAlert::APOL));
    spi.done();
}
