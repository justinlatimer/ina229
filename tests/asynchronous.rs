#![cfg(feature = "async")]

use embedded_hal::spi::{ErrorKind, ErrorType, Operation};
use embedded_hal_async::spi::SpiDevice;
use futures_lite::future::block_on;
use ina229::{Configuration, Error, INA229Async};

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
