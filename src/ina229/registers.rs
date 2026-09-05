use byteorder::{BigEndian, ByteOrder};

pub(super) const MAX_FRAME_LEN: usize = 4;

pub(super) trait ReadableRegister {
    type Value;

    const ADDRESS: u8;
    const FRAME_LEN: usize;

    fn decode(data: &[u8]) -> Self::Value;
}

pub(super) trait WritableRegister: ReadableRegister {
    fn encode(value: Self::Value, data: &mut [u8]);
}

pub(super) fn read_register_buffer<Register: ReadableRegister>() -> [u8; MAX_FRAME_LEN] {
    let mut buffer = [0; MAX_FRAME_LEN];
    buffer[0] = (Register::ADDRESS << 2) | 1;
    buffer
}

pub(super) fn write_register_buffer<Register: WritableRegister>(
    value: Register::Value,
) -> [u8; MAX_FRAME_LEN] {
    let mut buffer = [0; MAX_FRAME_LEN];
    buffer[0] = Register::ADDRESS << 2;
    Register::encode(value, &mut buffer[1..Register::FRAME_LEN]);
    buffer
}

fn decode_u16(data: &[u8]) -> u16 {
    BigEndian::read_u16(data)
}

fn encode_u16(value: u16, data: &mut [u8]) {
    BigEndian::write_u16(data, value);
}

fn decode_i16(data: &[u8]) -> i16 {
    BigEndian::read_i16(data)
}

fn encode_i16(value: i16, data: &mut [u8]) {
    BigEndian::write_i16(data, value);
}

fn decode_u24(data: &[u8]) -> u32 {
    BigEndian::read_u24(data)
}

fn decode_u20(data: &[u8]) -> u32 {
    decode_u24(data) >> 4
}

fn decode_i20(data: &[u8]) -> i32 {
    BigEndian::read_i24(data) >> 4
}

macro_rules! readable_register {
    ($name:ident, $address:expr, $frame_len:expr, $value:ty, $decode:path) => {
        pub(super) enum $name {}

        impl ReadableRegister for $name {
            type Value = $value;

            const ADDRESS: u8 = $address;
            const FRAME_LEN: usize = $frame_len;

            fn decode(data: &[u8]) -> Self::Value {
                $decode(data)
            }
        }
    };
}

macro_rules! read_write_register {
    (
        $name:ident,
        $address:expr,
        $frame_len:expr,
        $value:ty,
        $decode:path,
        $encode:path
    ) => {
        readable_register!($name, $address, $frame_len, $value, $decode);

        impl WritableRegister for $name {
            fn encode(value: Self::Value, data: &mut [u8]) {
                $encode(value, data);
            }
        }
    };
}

// CONFIG
read_write_register!(ConfigurationRegister, 0x00, 3, u16, decode_u16, encode_u16);
// SHUNT_CAL
read_write_register!(
    ShuntCalibrationRegister,
    0x02,
    3,
    u16,
    decode_u16,
    encode_u16
);
// VSHUNT
readable_register!(ShuntVoltageRegister, 0x04, 4, i32, decode_i20);
// VBUS
readable_register!(BusVoltageRegister, 0x05, 4, u32, decode_u20);
// DIETEMP
readable_register!(DieTemperatureRegister, 0x06, 3, i16, decode_i16);
// CURRENT
readable_register!(CurrentRegister, 0x07, 4, i32, decode_i20);
// POWER
readable_register!(PowerRegister, 0x08, 4, u32, decode_u24);
// DIAG_ALRT
read_write_register!(DiagAlertRegister, 0x0B, 3, u16, decode_u16, encode_u16);
// SOVL
read_write_register!(
    ShuntOverVoltageRegister,
    0x0C,
    3,
    i16,
    decode_i16,
    encode_i16
);
// SUVL
read_write_register!(
    ShuntUnderVoltageRegister,
    0x0D,
    3,
    i16,
    decode_i16,
    encode_i16
);
// BOVL
read_write_register!(BusOverVoltageRegister, 0x0E, 3, u16, decode_u16, encode_u16);
// BUVL
read_write_register!(
    BusUnderVoltageRegister,
    0x0F,
    3,
    u16,
    decode_u16,
    encode_u16
);
// TEMP_LIMIT
read_write_register!(
    TemperatureOverLimitRegister,
    0x10,
    3,
    i16,
    decode_i16,
    encode_i16
);
// PWR_LIMIT
read_write_register!(PowerOverLimitRegister, 0x11, 3, u16, decode_u16, encode_u16);
// MANUFACTURER_ID
readable_register!(ManufacturerIdRegister, 0x3E, 3, u16, decode_u16);
// DEVICE_ID
readable_register!(DeviceIdRegister, 0x3F, 3, u16, decode_u16);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_values_are_decoded() {
        assert_eq!(ConfigurationRegister::decode(&[0x12, 0x34]), 0x1234);
        assert_eq!(ShuntOverVoltageRegister::decode(&[0xFF, 0xFE]), -2);
        assert_eq!(PowerRegister::decode(&[0x12, 0x34, 0x56]), 0x123456);
        assert_eq!(BusVoltageRegister::decode(&[0xFF, 0xFF, 0xF0]), 0xFFFFF);
        assert_eq!(ShuntVoltageRegister::decode(&[0x00, 0x00, 0x00]), 0);
        assert_eq!(ShuntVoltageRegister::decode(&[0xFF, 0xFF, 0xF0]), -1);
        assert_eq!(
            ShuntVoltageRegister::decode(&[0x80, 0x00, 0x00]),
            -(1 << 19)
        );
        assert_eq!(
            ShuntVoltageRegister::decode(&[0x7F, 0xFF, 0xF0]),
            (1 << 19) - 1
        );
    }
}
