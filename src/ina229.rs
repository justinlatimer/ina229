//! The INA229 driver itself, and the values it hands back.

use core::result::Result;

use bitflags::bitflags;
use byteorder::{BigEndian, ByteOrder};
use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    spi::{Mode, MODE_1},
};

bitflags! {
    /// Configuration register contents.
    #[repr(C)]
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
    #[repr(C)]
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
    TOLT = 0x10,
    ManufacturerID = 0x3E,
    DeviceID = 0x3F,
}

enum Command {
    Read,
    Write,
}

/// Error type for INA229 commands.
#[derive(Debug)]
pub enum Error<SPIError, CSError> {
    /// The INA229 is not configured.
    NotConfigured,

    /// An error occured during an SPI transaction.
    SPIError(SPIError),

    /// An error occured toggling the chip select.
    ChipSelectError(CSError),
}

// Conversion constants
const BUS_VOLTAGE_UV_PER_LSB: f64 = 195.3125;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_0: f64 = 312.5;
const SHUNT_VOLTAGE_NV_PER_LSB_MODE_1: f64 = 78.125;
const TEMPERATURE_MC_PER_LSB: f64 = 7.8125;
const POWER_SCALING_FACTOR: f64 = 3.2;

// Alert threshold conversion constants
const SHUNT_THRESHOLD_UV_PER_LSB_MODE_0: f64 = 5.0;
const SHUNT_THRESHOLD_UV_PER_LSB_MODE_1: f64 = 1.25;
const TEMPERATURE_THRESHOLD_MC_PER_LSB: f64 = TEMPERATURE_MC_PER_LSB;

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

/// INA229 voltage/current/power monitor
pub struct INA229<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
    config: Option<Configuration>,
    current_lsb: Option<f64>,
}

impl<SPI, NCS, SPIError, CSError> INA229<SPI, NCS>
where
    SPI: Transfer<u8, Error = SPIError> + Write<u8, Error = SPIError>,
    NCS: OutputPin<Error = CSError>,
{
    /// Create a new instance of an INA229 device.
    pub fn new(spi: SPI, ncs: NCS) -> Self {
        INA229 {
            spi,
            ncs,
            config: None,
            current_lsb: None,
        }
    }

    /// Destroy the INA229 instance and return the SPI.
    pub fn release(self) -> (SPI, NCS) {
        (self.spi, self.ncs)
    }

    fn read_register_u16(&mut self, register: Register) -> Result<u16, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_u16(&buffer[1..3]);
        Ok(value)
    }

    fn write_register_u16(
        &mut self,
        register: Register,
        value: u16,
    ) -> Result<(), Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Write), 0x00, 0x00];
        BigEndian::write_u16_into(&[value], &mut buffer[1..3]);
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.write(&buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        Ok(())
    }

    fn read_register_i16(&mut self, register: Register) -> Result<i16, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_i16(&buffer[1..3]);
        Ok(value)
    }

    fn write_register_i16(
        &mut self,
        register: Register,
        value: i16,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_u16(register, value as u16)
    }

    fn read_register_u24(&mut self, register: Register) -> Result<u32, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_u24(&buffer[1..4]);
        Ok(value)
    }

    fn read_register_i24(&mut self, register: Register) -> Result<i32, Error<SPIError, CSError>> {
        let mut buffer = [get_frame(register, Command::Read), 0x00, 0x00, 0x00];
        self.ncs.set_low().map_err(Error::ChipSelectError)?;
        self.spi.transfer(&mut buffer).map_err(Error::SPIError)?;
        self.ncs.set_high().map_err(Error::ChipSelectError)?;
        let value = BigEndian::read_i24(&buffer[1..4]);
        Ok(value)
    }

    /// Sets the CONFIG register with the value provided.
    pub fn set_configuration(
        &mut self,
        configuration: Configuration,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_u16(Register::Configuration, configuration.bits())?;
        self.config = Some(configuration);
        Ok(())
    }

    /// Get the configuration.
    pub fn configuration(&mut self) -> Result<Configuration, Error<SPIError, CSError>> {
        self.read_register_u16(Register::Configuration)
            .map(Configuration::from_bits_truncate)
            .map(|config| {
                self.config = Some(config);
                config
            })
    }

    /// Configure ALERT pin behaviour by writing to the DIAG_ALERT register.
    pub fn set_alert_configuration(
        &mut self,
        configuration: DiagAlert,
    ) -> Result<(), Error<SPIError, CSError>> {
        let mask = DiagAlert::ALATCH | DiagAlert::CNVR | DiagAlert::SLOWALERT | DiagAlert::APOL;
        self.write_register_u16(Register::DiagAlert, (configuration & mask).bits())
    }

    /// Read the DIAG_ALERT register, giving the current ALERT pin configuration.
    /// Note: if [`DiagAlert::ALATCH`] is enabled, reading this register clears the ALERT pin.
    pub fn alert_configuration(&mut self) -> Result<DiagAlert, Error<SPIError, CSError>> {
        self.read_register_u16(Register::DiagAlert)
            .map(DiagAlert::from_bits_truncate)
    }

    /// Gets the value from the shunt calibration register.
    pub fn shunt_calibration(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::ShuntCalibration)
    }

    /// Sets the shunt calibration register to the value provided.
    pub fn set_shunt_calibration(&mut self, value: u16) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_u16(Register::ShuntCalibration, value)
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    pub fn calibrate(
        &mut self,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        if let Some(config) = self.config {
            let (current_lsb, value) =
                calculate_calibration_value(config, shunt_resistance, current_expected_max);
            self.set_shunt_calibration(value)?;
            self.current_lsb = Some(current_lsb);
            Ok(())
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Calculate the shunt calibration value and write to the shunt calibration register.
    pub fn configure_and_calibrate(
        &mut self,
        configuration: Configuration,
        shunt_resistance: f64,
        current_expected_max: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.set_configuration(configuration)
            .and_then(|_| self.calibrate(shunt_resistance, current_expected_max))
    }

    /// Get the raw bus voltage reading.
    pub fn bus_voltage_raw(&mut self) -> Result<i32, Error<SPIError, CSError>> {
        self.read_register_i24(Register::BusVoltage).map(|x| x >> 4) // 20bit value.
    }

    /// Get the bus voltage reading in microvolts.
    pub fn bus_voltage_microvolts(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        self.bus_voltage_raw()
            .map(|x| (x as f64) * BUS_VOLTAGE_UV_PER_LSB)
    }

    /// Get the raw shunt voltage reading.
    pub fn shunt_voltage_raw(&mut self) -> Result<i32, Error<SPIError, CSError>> {
        self.read_register_i24(Register::ShuntVoltage)
            .map(|x| x >> 4)
    }

    /// Get the shunt voltage reading in nanovolts.
    pub fn shunt_voltage_nanovolts(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        if let Some(config) = self.config {
            self.shunt_voltage_raw().map(|value| {
                if config.contains(Configuration::ADCRANGE) {
                    (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_1
                } else {
                    (value as f64) * SHUNT_VOLTAGE_NV_PER_LSB_MODE_0
                }
            })
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get the raw die temperature value.
    pub fn temperature_raw(&mut self) -> Result<i16, Error<SPIError, CSError>> {
        self.read_register_i16(Register::DieTemperature)
    }

    /// Get the die temperature in millidegrees Celsius.
    pub fn temperature_millidegrees_celsius(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        self.temperature_raw()
            .map(|x| (x as f64) * TEMPERATURE_MC_PER_LSB)
    }

    /// Get the raw value from the current register.
    pub fn current_raw(&mut self) -> Result<i32, Error<SPIError, CSError>> {
        self.read_register_i24(Register::Current).map(|x| x >> 4) // 20bit value.
    }

    /// Get the current reading in Amps.
    pub fn current_amps(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        if let Some(current_lsb) = self.current_lsb {
            self.current_raw().map(|x| (x as f64) * current_lsb)
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get the raw value from the power register
    pub fn power_raw(&mut self) -> Result<u32, Error<SPIError, CSError>> {
        self.read_register_u24(Register::Power)
    }

    /// Get the power reading in Watts.
    pub fn power_watts(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        if let Some(current_lsb) = self.current_lsb {
            self.power_raw()
                .map(|x| (x as f64) * current_lsb * POWER_SCALING_FACTOR)
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get Shunt threshold conversion given ADCRANGE.
    fn shunt_threshold_microvolts_per_lsb(&mut self) -> Result<f64, Error<SPIError, CSError>> {
        if let Some(config) = self.config {
            Ok(if config.contains(Configuration::ADCRANGE) {
                SHUNT_THRESHOLD_UV_PER_LSB_MODE_1
            } else {
                SHUNT_THRESHOLD_UV_PER_LSB_MODE_0
            })
        } else {
            Err(Error::NotConfigured)
        }
    }

    /// Get the raw shunt overvoltage threshold, from the SOVL register.
    pub fn shunt_overvoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError, CSError>> {
        self.read_register_i16(Register::SOVL)
    }

    /// Set the raw shunt overvoltage threshold, in the SOVL register.
    pub fn set_shunt_overvoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_i16(Register::SOVL, value)
    }

    /// Set the shunt overvoltage threshold in microvolts. Requires the INA229 to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub fn set_shunt_overvoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        let lsb = self.shunt_threshold_microvolts_per_lsb()?;
        self.set_shunt_overvoltage_threshold_raw((microvolts / lsb) as i16)
    }

    /// Get the raw shunt undervoltage threshold, from the SUVL register.
    pub fn shunt_undervoltage_threshold_raw(&mut self) -> Result<i16, Error<SPIError, CSError>> {
        self.read_register_i16(Register::SUVL)
    }

    /// Set the raw shunt undervoltage threshold, in the SUVL register.
    pub fn set_shunt_undervoltage_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_i16(Register::SUVL, value)
    }

    /// Set the shunt undervoltage threshold in microvolts. Requires the device to already
    /// be configured, as the conversion factor depends on the ADCRANGE setting.
    pub fn set_shunt_undervoltage_threshold_microvolts(
        &mut self,
        microvolts: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        let lsb = self.shunt_threshold_microvolts_per_lsb()?;
        self.set_shunt_undervoltage_threshold_raw((microvolts / lsb) as i16)
    }

    /// Get the raw temperature over-limit threshold, from the TEMP_LIMIT register.
    pub fn temperature_overlimit_threshold_raw(&mut self) -> Result<i16, Error<SPIError, CSError>> {
        self.read_register_i16(Register::TOLT)
    }

    /// Set the raw temperature over-limit threshold, in the TEMP_LIMIT register.
    pub fn set_temperature_overlimit_threshold_raw(
        &mut self,
        value: i16,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.write_register_i16(Register::TOLT, value)
    }

    /// Set the temperature over-limit threshold in millidegrees Celsius.
    pub fn set_temperature_overlimit_threshold_millidegrees_celsius(
        &mut self,
        millidegrees_celsius: f64,
    ) -> Result<(), Error<SPIError, CSError>> {
        self.set_temperature_overlimit_threshold_raw(
            (millidegrees_celsius / TEMPERATURE_THRESHOLD_MC_PER_LSB) as i16,
        )
    }

    /// Get the unique manufacturer identification number.
    pub fn manufacturer_id(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::ManufacturerID)
    }

    /// Get the unique die identification number.
    pub fn device_id(&mut self) -> Result<u16, Error<SPIError, CSError>> {
        self.read_register_u16(Register::DeviceID)
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
