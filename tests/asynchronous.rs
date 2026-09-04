#![cfg(feature = "async")]

use approx::assert_relative_eq;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_mock::eh1::spi::{Mock as SPIMock, Transaction as SPITransaction};
use futures_lite::future::block_on;
use ina229::{Configuration, Error, INA229Async, INA229};

#[test]
fn blocking_and_async_devices_can_coexist() {
    block_on(async {
        let blocking_expectations = [
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0xF9, 0x00, 0x00], vec![0x00, 0x54, 0x49]),
            SPITransaction::transaction_end(),
        ];
        let async_expectations = [
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0xFD, 0x00, 0x00], vec![0x00, 0x22, 0x90]),
            SPITransaction::transaction_end(),
        ];
        let mut blocking = INA229::new(SPIMock::new(&blocking_expectations));
        let mut asynchronous = INA229Async::new(SPIMock::new(&async_expectations));

        assert_eq!(blocking.manufacturer_id().unwrap(), 0x5449);
        assert_eq!(asynchronous.device_id().await.unwrap(), 0x2290);

        blocking.release().done();
        asynchronous.release().done();
    });
}

#[test]
fn async_register_access_and_stateful_conversions_work() {
    block_on(async {
        let spi_expectations = [
            SPITransaction::transaction_start(),
            SPITransaction::write_vec(vec![0x00, 0x00, 0x00]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::write_vec(vec![0x08, 0x0F, 0xD2]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0xF9, 0x00, 0x00], vec![0x00, 0x54, 0x49]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0x19, 0x00, 0x00], vec![0x00, 0x0C, 0x80]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(
                vec![0x1D, 0x00, 0x00, 0x00],
                vec![0x00, 0x4C, 0xCC, 0xC0],
            ),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(
                vec![0x21, 0x00, 0x00, 0x00],
                vec![0x00, 0x48, 0x00, 0x0C],
            ),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::write_vec(vec![0x34, 0x81, 0x70]),
            SPITransaction::transaction_end(),
        ];
        let spi = SPIMock::new(&spi_expectations);
        let mut ina229 = INA229Async::new(spi);

        ina229
            .configure_and_calibrate(Configuration::empty(), 0.0162, 10.0)
            .await
            .expect("configuration and calibration to succeed");

        assert_eq!(
            ina229
                .manufacturer_id()
                .await
                .expect("manufacturer ID to be returned"),
            0x5449
        );
        assert_eq!(
            ina229
                .temperature_raw()
                .await
                .expect("temperature to be returned"),
            3200
        );
        assert_relative_eq!(
            ina229.current_amps().await.expect("current to be returned"),
            6.0,
            max_relative = 0.00001
        );
        assert_relative_eq!(
            ina229.power_watts().await.expect("power to be returned"),
            288.0,
            max_relative = 0.00001
        );
        ina229
            .set_shunt_undervoltage_threshold_raw(-32400)
            .await
            .expect("threshold to be written");

        let mut spi = ina229.release();
        spi.done();
    });
}

#[test]
fn async_configuration_and_alert_access_work() {
    block_on(async {
        let spi_expectations = [
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0x01, 0x00, 0x00], vec![0x00, 0x00, 0x10]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::write_vec(vec![0x2C, 0x80, 0x00]),
            SPITransaction::transaction_end(),
            SPITransaction::transaction_start(),
            SPITransaction::transfer_in_place(vec![0x2D, 0x00, 0x00], vec![0x00, 0x90, 0x40]),
            SPITransaction::transaction_end(),
        ];
        let mut ina229 = INA229Async::new(SPIMock::new(&spi_expectations));

        assert_eq!(
            ina229.configuration().await.unwrap(),
            Configuration::ADCRANGE
        );
        ina229
            .set_alert_configuration(ina229::DiagAlert::ALATCH | ina229::DiagAlert::TMPOL)
            .await
            .unwrap();
        let alert = ina229.alert_configuration().await.unwrap();
        assert!(alert.contains(ina229::DiagAlert::ALATCH));
        assert!(alert.contains(ina229::DiagAlert::SHNTOL));

        ina229.release().done();
    });
}

#[derive(Default)]
struct RecordingSPI {
    accesses: Vec<(u8, usize)>,
    writes: Vec<Vec<u8>>,
}

impl ErrorType for RecordingSPI {
    type Error = ErrorKind;
}

impl SpiDevice for RecordingSPI {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for operation in operations {
            match operation {
                Operation::Read(_) | Operation::DelayNs(_) => {}
                Operation::Write(buffer) => {
                    self.accesses.push((buffer[0], buffer.len()));
                    self.writes.push(buffer.to_vec());
                }
                Operation::TransferInPlace(buffer) => {
                    self.accesses.push((buffer[0], buffer.len()));
                }
                Operation::Transfer(_, write) => {
                    self.accesses.push((write[0], write.len()));
                }
            }
        }
        Ok(())
    }
}

#[test]
fn all_async_methods_access_expected_registers() {
    block_on(async {
        let mut ina229 = INA229Async::new(RecordingSPI::default());

        ina229
            .set_configuration(Configuration::empty())
            .await
            .unwrap();
        ina229.configuration().await.unwrap();
        ina229
            .set_alert_configuration(ina229::DiagAlert::empty())
            .await
            .unwrap();
        ina229.alert_configuration().await.unwrap();
        ina229.shunt_calibration().await.unwrap();
        ina229.set_shunt_calibration(0x1234).await.unwrap();
        ina229.calibrate(0.0162, 10.0).await.unwrap();
        ina229
            .configure_and_calibrate(Configuration::empty(), 0.0162, 10.0)
            .await
            .unwrap();

        ina229.bus_voltage_raw().await.unwrap();
        ina229.bus_voltage_microvolts().await.unwrap();
        ina229.shunt_voltage_raw().await.unwrap();
        ina229.shunt_voltage_nanovolts().await.unwrap();
        ina229.temperature_raw().await.unwrap();
        ina229.temperature_millidegrees_celsius().await.unwrap();
        ina229.current_raw().await.unwrap();
        ina229.current_amps().await.unwrap();
        ina229.power_raw().await.unwrap();
        ina229.power_watts().await.unwrap();

        ina229.shunt_overvoltage_threshold_raw().await.unwrap();
        ina229
            .set_shunt_overvoltage_threshold_raw(123)
            .await
            .unwrap();
        ina229
            .set_shunt_overvoltage_threshold_microvolts(10.0)
            .await
            .unwrap();
        ina229.shunt_undervoltage_threshold_raw().await.unwrap();
        ina229
            .set_shunt_undervoltage_threshold_raw(-123)
            .await
            .unwrap();
        ina229
            .set_shunt_undervoltage_threshold_microvolts(-10.0)
            .await
            .unwrap();
        ina229.bus_overvoltage_threshold_raw().await.unwrap();
        ina229
            .set_bus_overvoltage_threshold_raw(1000)
            .await
            .unwrap();
        ina229
            .set_bus_overvoltage_threshold_millivolts(3125.0)
            .await
            .unwrap();
        ina229.bus_undervoltage_threshold_raw().await.unwrap();
        ina229
            .set_bus_undervoltage_threshold_raw(2000)
            .await
            .unwrap();
        ina229
            .set_bus_undervoltage_threshold_millivolts(6250.0)
            .await
            .unwrap();
        ina229.temperature_overlimit_threshold_raw().await.unwrap();
        ina229
            .set_temperature_overlimit_threshold_raw(-100)
            .await
            .unwrap();
        ina229
            .set_temperature_overlimit_threshold_millidegrees_celsius(781.25)
            .await
            .unwrap();
        ina229.power_overlimit_threshold_raw().await.unwrap();
        ina229.set_power_overlimit_threshold_raw(300).await.unwrap();
        ina229
            .set_power_overlimit_threshold_watts(1.5625)
            .await
            .unwrap();
        ina229.manufacturer_id().await.unwrap();
        ina229.device_id().await.unwrap();

        let spi = ina229.release();
        assert_eq!(
            spi.accesses,
            [
                (0x00, 3),
                (0x01, 3),
                (0x2C, 3),
                (0x2D, 3),
                (0x09, 3),
                (0x08, 3),
                (0x08, 3),
                (0x00, 3),
                (0x08, 3),
                (0x15, 4),
                (0x15, 4),
                (0x11, 4),
                (0x11, 4),
                (0x19, 3),
                (0x19, 3),
                (0x1D, 4),
                (0x1D, 4),
                (0x21, 4),
                (0x21, 4),
                (0x31, 3),
                (0x30, 3),
                (0x30, 3),
                (0x35, 3),
                (0x34, 3),
                (0x34, 3),
                (0x39, 3),
                (0x38, 3),
                (0x38, 3),
                (0x3D, 3),
                (0x3C, 3),
                (0x3C, 3),
                (0x41, 3),
                (0x40, 3),
                (0x40, 3),
                (0x45, 3),
                (0x44, 3),
                (0x44, 3),
                (0xF9, 3),
                (0xFD, 3),
            ]
        );
        assert_eq!(
            spi.writes,
            [
                vec![0x00, 0x00, 0x00],
                vec![0x2C, 0x00, 0x00],
                vec![0x08, 0x12, 0x34],
                vec![0x08, 0x0F, 0xD2],
                vec![0x00, 0x00, 0x00],
                vec![0x08, 0x0F, 0xD2],
                vec![0x30, 0x00, 0x7B],
                vec![0x30, 0x00, 0x02],
                vec![0x34, 0xFF, 0x85],
                vec![0x34, 0xFF, 0xFE],
                vec![0x38, 0x03, 0xE8],
                vec![0x38, 0x03, 0xE8],
                vec![0x3C, 0x07, 0xD0],
                vec![0x3C, 0x07, 0xD0],
                vec![0x40, 0xFF, 0x9C],
                vec![0x40, 0x00, 0x64],
                vec![0x44, 0x01, 0x2C],
                vec![0x44, 0x00, 0x64],
            ]
        );
    });
}

#[test]
fn async_validation_happens_before_spi_access() {
    block_on(async {
        let spi_expectations = [];
        let spi = SPIMock::new(&spi_expectations);
        let mut ina229 = INA229Async::new(spi);

        let result = ina229.set_bus_overvoltage_threshold_raw(0x8000).await;

        assert!(matches!(result, Err(Error::InvalidRawValue(0x8000))));
        let mut spi = ina229.release();
        spi.done();
    });
}

struct FailingSPI;

impl ErrorType for FailingSPI {
    type Error = ErrorKind;
}

impl SpiDevice for FailingSPI {
    async fn transaction(
        &mut self,
        _operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        Err(ErrorKind::Other)
    }
}

#[test]
fn async_spi_errors_are_mapped() {
    block_on(async {
        let mut ina229 = INA229Async::new(FailingSPI);

        let result = ina229.manufacturer_id().await;

        assert_eq!(result, Err(Error::SPIError(ErrorKind::Other)));

        let result = ina229
            .set_alert_configuration(ina229::DiagAlert::APOL)
            .await;

        assert_eq!(result, Err(Error::SPIError(ErrorKind::Other)));
    });
}

struct FailOnSecondSPI {
    transaction_count: usize,
}

impl ErrorType for FailOnSecondSPI {
    type Error = ErrorKind;
}

impl SpiDevice for FailOnSecondSPI {
    async fn transaction(
        &mut self,
        _operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.transaction_count += 1;
        if self.transaction_count == 2 {
            Err(ErrorKind::Other)
        } else {
            Ok(())
        }
    }
}

#[test]
fn async_configure_and_calibrate_maps_second_write_error() {
    block_on(async {
        let spi = FailOnSecondSPI {
            transaction_count: 0,
        };
        let mut ina229 = INA229Async::new(spi);

        let result = ina229
            .configure_and_calibrate(Configuration::empty(), 0.0162, 10.0)
            .await;

        assert_eq!(result, Err(Error::SPIError(ErrorKind::Other)));
        assert_eq!(ina229.release().transaction_count, 2);
    });
}
