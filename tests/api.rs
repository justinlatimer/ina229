macro_rules! api_cases {
    () => {
        mod alert {
            use super::*;
            include!("api/cases/alert.rs");
        }

        mod calibration {
            use super::*;
            include!("api/cases/calibration.rs");
        }

        mod configuration {
            use super::*;
            include!("api/cases/configuration.rs");
        }

        mod current_power {
            use super::*;
            include!("api/cases/current_power.rs");
        }

        mod identification {
            use super::*;
            include!("api/cases/identification.rs");
        }

        mod temperature {
            use super::*;
            include!("api/cases/temperature.rs");
        }

        mod voltage {
            use super::*;
            include!("api/cases/voltage.rs");
        }
    };
}

mod blocking {
    use approx::assert_relative_eq;
    use embedded_hal_mock::eh1::spi::{Mock as SPIMock, Transaction as SPITransaction};
    use ina229::{Configuration, DiagAlert, INA229 as Driver};

    macro_rules! invoke {
        ($operation:expr) => {
            $operation
        };
    }

    api_cases!();
}

#[cfg(feature = "async")]
mod asynchronous {
    use approx::assert_relative_eq;
    use embedded_hal_mock::eh1::spi::{Mock as SPIMock, Transaction as SPITransaction};
    use ina229::{Configuration, DiagAlert, INA229Async as Driver};

    macro_rules! invoke {
        ($operation:expr) => {
            futures_lite::future::block_on($operation)
        };
    }

    api_cases!();
}
