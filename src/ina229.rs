//! The INA229 driver itself, and the values it hands back.

use core::fmt::{self, Debug, Display};
use core::result::Result;

use bitflags::bitflags;
use byteorder::{BigEndian, ByteOrder};
use embedded_hal::spi::{Mode, MODE_1};

#[cfg(feature = "async")]
mod asynchronous;
mod blocking;

bitflags! {
    /// Configuration register contents.
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    #[repr(transparent)]
    pub struct Configuration: u16 {
        /// Reset Bit. Setting this bit to '1' generates a system reset that is the same as power-on reset.
        /// Resets all registers to default values.
        /// 0h = Normal Operation
        /// 1h = System Reset sets registers to default values
        /// This bit self-clears.
        /// Default: 0.
        const RST       = 0b1000_0000_0000_0000;

        /// Resets the contents of accumulation registers ENERGY and CHARGE to 0.
        /// 0h = Normal Operation
        /// 1h = Clears registers to default values for ENERGY and CHARGE registers
        /// Default: 0.
        const RSTACC    = 0b0100_0000_0000_0000;

        /// Sets the Delay for initial ADC conversion in steps of 2 ms.
        /// 0h = 0 s
        /// 1h = 2 ms
        /// FFh = 510 ms
        /// Default: 0.
        const CONVDLY   = 0b0011_1111_1100_0000;

        /// Enables temperature compensation of an external shunt
        /// 0h = Shunt Temperature Compensation Disabled
        /// 1h = Shunt Temperature Compensation Enabled
        /// Default: 0.
        const TEMPCOMP  = 0b0000_0000_0010_0000;

        /// Shunt full scale range selection across IN+ and IN–.
        /// 0h = ±163.84 mV
        /// 1h = ± 40.96 mV
        /// Default: 0.
        const ADCRANGE  = 0b0000_0000_0001_0000;

        /// Reserved. Always reads 0.
        const RESERVED  = 0b0000_0000_0000_1111;
    }
}

bitflags! {
    /// Diagnostic Flags and Alert register contents.
    ///
    /// The top four bits (ALATCH, CNVR, SLOWALERT, APOL) configure the behaviour of the
    /// open-drain ALERT pin. The remaining bits are read-only status/diag flags that
    /// are set by the device. When [`DiagAlert::ALATCH`] is enabled, reading this reg
    /// clears any latched flags (see INA229 datasheet, Table 7-16).
    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    #[repr(transparent)]
    pub struct DiagAlert: u16 {
        /// Alert Latch Enable
        /// 0h = Transparent: the ALERT pin and flag bit clear as soon as the fault clears.
        /// 1h = Latched: the ALERT pin and flag bit stay active until this register is read.
        /// Default: 0.
        const ALATCH    = 0b1000_0000_0000_0000;

        /// Configures the ALERT pin to assert when the Conversion Ready flag (CNVRF) is set,
        /// indicating a conversion cycle has completed.
        /// 0h = Disable conversion ready flag on ALERT pin
        /// 1h = Enable conversion ready flag on ALERT pin
        /// Default: 0.
        const CNVR      = 0b0100_0000_0000_0000;

        /// When enabled, alert comparisons are made against the averaged output value
        /// instead of the raw (non-averaged) ADC value.
        /// 0h = Compare against non-averaged (ADC) value
        /// 1h = Compare against averaged value
        /// Default: 0.
        const SLOWALERT = 0b0010_0000_0000_0000;

        /// Sets the Alert pin polarity. The ALERT pin is always open-drain.
        /// 0h = Normal (active-low)
        /// 1h = Inverted (active-high)
        /// Default: 0.
        const APOL      = 0b0001_0000_0000_0000;

        /// Bit is set if the 40-bit ENERGY register has overflowed. Bit is Read-Only.
        /// Clears when ENERGY is read.
        /// 0h = Normal
        /// 1h = Overflow
        /// Default: 0.
        const ENERGYOF  = 0b0000_1000_0000_0000;

        /// Bit is set if the 40-bit CHARGE register has overflowed. Bit is Read-Only.
        /// Clears when CHARGE is read.
        /// 0h = Normal
        /// 1h = Overflow
        /// Default: 0.
        const CHARGEOF  = 0b0000_0100_0000_0000;

        /// Bit is set if an arithmetic operation caused an overflow error. Current and power
        /// data may be invalid. Cleared by triggering another conversion or by clearing the
        /// accumulators with the RSTACC bit. Bit is Read-Only.
        /// 0h = Normal
        /// 1h = Overflow
        /// Default: 0.
        const MATHOF    = 0b0000_0010_0000_0000;

        /// Bit is set if the die temperature exceeds the threshold in the TEMP_LIMIT register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Over Temperature Event
        /// Default: 0.
        const TMPOL     = 0b0000_0000_1000_0000;

        /// Bit is set if the shunt voltage exceeds the threshold in the SOVL register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Over Shunt Voltage Event
        /// Default: 0.
        const SHNTOL    = 0b0000_0000_0100_0000;

        /// Bit is set if the shunt voltage falls below the threshold in the SUVL register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Under Shunt Voltage Event
        /// Default: 0.
        const SHNTUL    = 0b0000_0000_0010_0000;

        /// Bit is set if the bus voltage exceeds the threshold in the BOVL register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Bus Over-Limit Event
        /// Default: 0.
        const BUSOL     = 0b0000_0000_0001_0000;

        /// Bit is set if the bus voltage falls below the threshold in the BUVL register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Bus Under-Limit Event
        /// Default: 0.
        const BUSUL     = 0b0000_0000_0000_1000;

        /// Bit is set if the power measurement exceeds the threshold in the PWR_LIMIT register.
        /// When ALATCH =1 this bit is cleared by reading this register.
        /// 0h = Normal
        /// 1h = Power Over-Limit Event
        /// Default: 0.
        const POL       = 0b0000_0000_0000_0100;

        /// Bit is set when an ADC conversion cycle has completed.
        /// When ALATCH =1 this bit is cleared by reading this register or by starting a new
        /// triggered conversion.
        /// 0h = Normal
        /// 1h = Conversion is complete
        /// Default: 0.
        const CNVRF     = 0b0000_0000_0000_0010;

        /// Reads 1h during normal operation. Reads 0h if a checksum error is detected in
        /// the device's non-volatile trim memory.
        /// 0h = Memory Checksum Error
        /// 1h = Normal operation
        /// Default: 1.
        const MEMSTAT   = 0b0000_0000_0000_0001;
    }
}

/// The SPI mode for the INA229.
pub const MODE: Mode = MODE_1;

#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum Register {
    Configuration = 0x00,
    ShuntCalibration = 0x02,
    ShuntVoltage = 0x04,
    BusVoltage = 0x05,
    DieTemperature = 0x06,
    Current = 0x07,
    Power = 0x08,
    DiagAlert = 0x0B,
    SOVL = 0x0C,
    SUVL = 0x0D,
    BOVL = 0x0E,
    BUVL = 0x0F,
    TOLT = 0x10,
    POLT = 0x11,
    ManufacturerID = 0x3E,
    DeviceID = 0x3F,
}

enum Command {
    Read,
    Write,
}

/// Error type for INA229 commands.
#[derive(Debug, Clone, PartialEq)]
pub enum Error<SPIError> {
    /// The INA229 is not configured.
    NotConfigured,

    /// An error occured during an SPI transaction.
    SPIError(SPIError),

    /// The raw value provided doesn't fit the register being written to.
    InvalidRawValue(u16),
}

impl<SPIError> Display for Error<SPIError>
where
    SPIError: Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::NotConfigured => write!(f, "INA229 is not configured"),
            Error::SPIError(error) => write!(f, "SPI transaction failed: {}", error),
            Error::InvalidRawValue(value) => write!(f, "invalid raw value: {}", value),
        }
    }
}

impl<SPIError> core::error::Error for Error<SPIError> where SPIError: Debug + Display {}

// Conversion constants
const BUS_VOLTAGE_UV_PER_LSB: f64 = 195.3125;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_0: f64 = 312.5;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_1: f64 = 78.125;
const TEMPERATURE_MC_PER_LSB: f64 = 7.8125;
const POWER_SCALING_FACTOR: f64 = 3.2;

// Alert threshold conversion constants
const BUS_THRESHOLD_MV_PER_LSB: f64 = 3.125;
const SHUNT_THRESHOLD_UV_PER_LSB_MODE_0: f64 = 5.0;
const SHUNT_THRESHOLD_UV_PER_LSB_MODE_1: f64 = 1.25;
const TEMPERATURE_THRESHOLD_MC_PER_LSB: f64 = TEMPERATURE_MC_PER_LSB;
const POWER_THRESHOLD_SCALING_FACTOR: f64 = 256.0;

// Register maximum raw values
const BUS_THRESHOLD_MAX_RAW: u16 = 0x7FFF; // From Datasheet (Tab. 7-19, 7-20), bit-15 reserved
const SHUNT_CALIBRATION_MAX_RAW: u16 = 0x7FFF; // From Datasheet (Tab. 7-7), bit-15 reserved

// Calibration constants
const DENOMINATOR: f64 = (1 << 19) as f64; // From Datasheet, 2^19
const INTERNAL_SCALING: f64 = 13107200000.0; // From Datasheet, 13107.2 * 10^6

#[inline(always)]
fn calculate_calibration_value(
    configuration: Configuration,
    shunt_resistance: f64,
    current_expected_max: f64,
) -> (f64, u16) {
    let scale = if configuration.contains(Configuration::ADCRANGE) {
        4.0
    } else {
        1.0
    };
    let current_lsb = calculate_current_lsb(current_expected_max);
    let shunt_cal = INTERNAL_SCALING * current_lsb * shunt_resistance * scale;
    (current_lsb, shunt_cal as u16)
}

#[inline(always)]
fn calculate_current_lsb(current_expected_max: f64) -> f64 {
    current_expected_max / DENOMINATOR
}

fn read_register_buffer<const LENGTH: usize>(register: Register) -> [u8; LENGTH] {
    let mut buffer = [0; LENGTH];
    buffer[0] = get_frame(register, Command::Read);
    buffer
}

fn write_register_u16_buffer(register: Register, value: u16) -> [u8; 3] {
    let mut buffer = [get_frame(register, Command::Write), 0x00, 0x00];
    BigEndian::write_u16_into(&[value], &mut buffer[1..3]);
    buffer
}

fn read_u16(buffer: &[u8]) -> u16 {
    BigEndian::read_u16(buffer)
}

fn read_i16(buffer: &[u8]) -> i16 {
    BigEndian::read_i16(buffer)
}

fn read_u24(buffer: &[u8]) -> u32 {
    BigEndian::read_u24(buffer)
}

fn read_i24(buffer: &[u8]) -> i32 {
    BigEndian::read_i24(buffer)
}

fn bus_voltage_microvolts_from_raw(value: i32) -> f64 {
    (value as f64) * BUS_VOLTAGE_UV_PER_LSB
}

fn shunt_voltage_nanovolts_from_raw(configuration: Configuration, value: i32) -> f64 {
    if configuration.contains(Configuration::ADCRANGE) {
        (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_1
    } else {
        (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_0
    }
}

fn temperature_millidegrees_celsius_from_raw(value: i16) -> f64 {
    (value as f64) * TEMPERATURE_MC_PER_LSB
}

fn current_amps_from_raw(current_lsb: f64, value: i32) -> f64 {
    (value as f64) * current_lsb
}

fn power_watts_from_raw(current_lsb: f64, value: u32) -> f64 {
    (value as f64) * current_lsb * POWER_SCALING_FACTOR
}

fn measurement_raw_from_register(value: i32) -> i32 {
    value >> 4
}

fn alert_configuration_raw(configuration: DiagAlert) -> u16 {
    let mask = DiagAlert::ALATCH | DiagAlert::CNVR | DiagAlert::SLOWALERT | DiagAlert::APOL;
    (configuration & mask).bits()
}

fn shunt_threshold_microvolts_per_lsb(configuration: Configuration) -> f64 {
    if configuration.contains(Configuration::ADCRANGE) {
        SHUNT_THRESHOLD_UV_PER_LSB_MODE_1
    } else {
        SHUNT_THRESHOLD_UV_PER_LSB_MODE_0
    }
}

fn shunt_threshold_raw(configuration: Configuration, microvolts: f64) -> i16 {
    (microvolts / shunt_threshold_microvolts_per_lsb(configuration)) as i16
}

fn bus_threshold_raw(millivolts: f64) -> u16 {
    (millivolts / BUS_THRESHOLD_MV_PER_LSB) as u16
}

fn temperature_threshold_raw(millidegrees_celsius: f64) -> i16 {
    (millidegrees_celsius / TEMPERATURE_THRESHOLD_MC_PER_LSB) as i16
}

fn power_threshold_raw(current_lsb: f64, watts: f64) -> u16 {
    let lsb = POWER_THRESHOLD_SCALING_FACTOR * current_lsb * POWER_SCALING_FACTOR;
    (watts / lsb) as u16
}

fn validate_raw_value<SPIError>(value: u16, maximum: u16) -> Result<(), Error<SPIError>> {
    if value > maximum {
        Err(Error::InvalidRawValue(value))
    } else {
        Ok(())
    }
}

#[derive(Default)]
struct State {
    config: Option<Configuration>,
    current_lsb: Option<f64>,
}

impl State {
    fn configuration(&self) -> Option<Configuration> {
        self.config
    }

    fn set_configuration(&mut self, configuration: Configuration) {
        self.config = Some(configuration);
    }

    fn calibration_value(
        &self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Option<(f64, u16)> {
        self.config.map(|configuration| {
            calculate_calibration_value(configuration, shunt_resistance, current_expected_max)
        })
    }

    fn set_current_lsb(&mut self, current_lsb: f64) {
        self.current_lsb = Some(current_lsb);
    }

    fn current_lsb(&self) -> Option<f64> {
        self.current_lsb
    }

    fn shunt_threshold_raw(&self, microvolts: f64) -> Option<i16> {
        self.config
            .map(|configuration| shunt_threshold_raw(configuration, microvolts))
    }

    fn power_threshold_raw(&self, watts: f64) -> Option<u16> {
        self.current_lsb
            .map(|current_lsb| power_threshold_raw(current_lsb, watts))
    }
}

/// Blocking INA229 voltage/current/power monitor.
///
/// # Example
///
/// ```no_run
/// # fn example<SPI>(spi: SPI) -> Result<(), ina229::Error<SPI::Error>>
/// # where
/// #     SPI: embedded_hal::spi::SpiDevice<u8>,
/// # {
/// let mut monitor = ina229::INA229::new(spi);
/// let voltage = monitor.bus_voltage_microvolts()?;
/// # let _ = voltage;
/// # Ok(())
/// # }
/// ```
pub struct INA229<SPI> {
    spi: SPI,
    state: State,
}

impl<SPI> INA229<SPI> {
    /// Create a new instance of an INA229 device.
    pub fn new(spi: SPI) -> Self {
        INA229 {
            spi,
            state: State::default(),
        }
    }

    /// Destroy the INA229 instance and return the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }
}

/// Asynchronous INA229 voltage/current/power monitor.
///
/// This type is available when the `async` Cargo feature is enabled.
///
/// # Example
///
/// ```no_run
/// # #[cfg(feature = "async")]
/// # async fn example<SPI>(spi: SPI) -> Result<(), ina229::Error<SPI::Error>>
/// # where
/// #     SPI: embedded_hal_async::spi::SpiDevice<u8>,
/// # {
/// let mut monitor = ina229::INA229Async::new(spi);
/// let voltage = monitor.bus_voltage_microvolts().await?;
/// # let _ = voltage;
/// # Ok(())
/// # }
/// ```
#[cfg(feature = "async")]
pub struct INA229Async<SPI> {
    spi: SPI,
    state: State,
}

#[cfg(feature = "async")]
impl<SPI> INA229Async<SPI> {
    /// Create a new asynchronous INA229 device.
    pub fn new(spi: SPI) -> Self {
        INA229Async {
            spi,
            state: State::default(),
        }
    }

    /// Destroy the asynchronous INA229 instance and return the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }
}

fn get_frame(register: Register, command: Command) -> u8 {
    let frame = (register as u8) << 2u8;
    match command {
        Command::Write => frame & !0b00000001,
        Command::Read => frame | 0b00000001,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        calculate_calibration_value, calculate_current_lsb, get_frame, Command, Configuration,
        Register,
    };
    use approx::assert_relative_eq;

    #[test]
    fn get_frame_manufacturer_read() {
        let result = get_frame(Register::ManufacturerID, Command::Read);
        assert_eq!(result, 0b1111_1001);
    }

    #[test]
    fn get_frame_device_read() {
        let result = get_frame(Register::DeviceID, Command::Read);
        assert_eq!(result, 0b1111_1101);
    }

    #[test]
    fn calculate_current_lsb_works() {
        let lsb = calculate_current_lsb(10.0); // 10 Amps
        assert_relative_eq!(lsb, 0.0000190735, max_relative = 0.00001);
    }

    #[test]
    fn calculate_calibration_value_works() {
        let (_, value) =
            calculate_calibration_value(Configuration::from_bits_truncate(0), 0.0162, 10.0);
        assert_eq!(value, 4050);
    }
}
