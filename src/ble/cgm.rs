//! Continuous Glucose Monitoring (CGM) types and helpers.
//!
//! Defines standard Bluetooth CGM Service UUIDs and data structures
//! for parsing CGM measurements received from a peripheral.

/// CGM Service UUID (0x181F)
pub const BT_UUID_CGMS_VAL: u16 = 0x181F;

/// CGM Measurement Characteristic UUID (0x2AA7)
pub const BT_UUID_CGM_MEASUREMENT_VAL: u16 = 0x2AA7;

/// CGM Feature Characteristic UUID (0x2AA8)
pub const BT_UUID_CGM_FEATURE_VAL: u16 = 0x2AA8;

/// CGM Status Characteristic UUID (0x2AA9)
pub const BT_UUID_CGM_STATUS_VAL: u16 = 0x2AA9;

/// GATT Client Characteristic Configuration Descriptor UUID (0x2902)
pub const BT_UUID_GATT_CCC_VAL: u16 = 0x2902;

/// BT_UUID_TYPE_16 from Zephyr (enum value 0)
pub const BT_UUID_TYPE_16: u8 = 0x00;

/// Encode a 16-bit UUID into the Zephyr `struct bt_uuid_16` wire format.
///
/// The Zephyr struct layout is:
/// ```c
/// struct bt_uuid_16 {
///     struct bt_uuid uuid;  // 1 byte: type
///     uint16_t val;         // 2 bytes: little-endian value
/// };
/// ```
///
/// Returns a 4-byte array matching the C `struct bt_uuid_16` layout:
/// `[type, padding(0x00), val_lo, val_hi]`.
pub fn encode_uuid_16(uuid_val: u16) -> [u8; 4] {
    let le = uuid_val.to_le_bytes();
    [BT_UUID_TYPE_16, 0x00, le[0], le[1]]
}

/// SFLOAT (Short Float) from IEEE 11073-20601.
///
/// 16-bit value: 4-bit exponent (signed) + 12-bit mantissa (signed).
#[derive(Debug, Clone, Copy)]
pub struct SFloat(pub u16);

impl SFloat {
    /// Decode the SFLOAT into an `f32` value.
    pub fn to_f32(self) -> f32 {
        let raw = self.0;
        let mut mantissa = (raw & 0x0FFF) as i16;
        let mut exponent = ((raw >> 12) & 0x0F) as i8;

        // Sign-extend mantissa from 12 bits
        if mantissa & 0x0800 != 0 {
            mantissa |= 0xF000_u16 as i16;
        }
        // Sign-extend exponent from 4 bits
        if exponent & 0x08 != 0 {
            exponent |= 0xF0_u8 as i8;
        }

        // Compute 10^exponent without f32::powi (unavailable in no_std)
        let mut result = mantissa as f32;
        if exponent > 0 {
            for _ in 0..exponent {
                result *= 10.0;
            }
        } else if exponent < 0 {
            for _ in 0..(-exponent) {
                result /= 10.0;
            }
        }
        result
    }
}

/// Parsed CGM Measurement notification data.
#[derive(Debug, Clone)]
pub struct CgmMeasurement {
    /// Raw SFLOAT glucose concentration
    pub glucose_raw: SFloat,
    /// Time offset in seconds from session start
    pub time_offset: u16,
    /// Flags byte
    pub flags: u8,
}

impl CgmMeasurement {
    /// Parse a CGM Measurement from raw notification data.
    ///
    /// The CGM Measurement structure:
    /// - Byte 0: Size
    /// - Byte 1: Flags
    /// - Bytes 2-3: Glucose Concentration (SFLOAT, little-endian)
    /// - Bytes 4-5: Time Offset (little-endian)
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 6 {
            return None;
        }
        let flags = data[1];
        let glucose_raw = SFloat(u16::from_le_bytes([data[2], data[3]]));
        let time_offset = u16::from_le_bytes([data[4], data[5]]);

        Some(Self {
            glucose_raw,
            time_offset,
            flags,
        })
    }

    /// Get glucose concentration as mg/dL (f32).
    pub fn glucose_mg_dl(&self) -> f32 {
        self.glucose_raw.to_f32()
    }
}
