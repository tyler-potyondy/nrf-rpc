//! Basic BLE data types and non-CBOR buffer encoders.
//!
//! These mirror a subset of the Zephyr Bluetooth types used by the nRF RPC
//! client, but are kept small and serialization-focused.

/// Bluetooth LE advertising/scan response data element.
///
/// This mirrors the logical contents of Zephyr's `struct bt_data`:
/// a type byte and an associated payload.
pub struct BtData<'a> {
    pub type_: u8,
    pub data: &'a [u8],
}

/// Standard advertising data type values (subset).
pub const BT_LE_AD_GENERAL: u8 = 0x02;
pub const BT_LE_AD_NO_BREDR: u8 = 0x04;

impl<'a> BtData<'a> {
    /// Construct a flags AD element.
    pub fn flags(flags: &'a [u8]) -> Self {
        Self {
            type_: 0x01, // Flags AD type
            data: flags,
        }
    }

    /// Construct a complete device name AD element.
    pub fn name_complete(name: &'a [u8]) -> Self {
        // 0x09 = Complete Local Name
        Self {
            type_: 0x09,
            data: name,
        }
    }

    /// Encode a slice of `BtData` into a flat, non-CBOR byte buffer.
    ///
    /// Layout per element:
    /// - 1 byte: type
    /// - 2 bytes: data length (little-endian)
    /// - N bytes: data
    ///
    /// Returns the number of bytes written into `out`.
    pub fn encode_list_into(list: &[BtData<'a>], out: &mut [u8]) -> usize {
        let mut pos = 0;

        for item in list {
            // Check there is room for type + length + data.
            let needed = 1 + 2 + item.data.len();
            if pos + needed > out.len() {
                break;
            }

            out[pos] = item.type_;
            pos += 1;

            let len = item.data.len() as u16;
            let len_bytes = len.to_le_bytes();
            out[pos..pos + 2].copy_from_slice(&len_bytes);
            pos += 2;

            out[pos..pos + item.data.len()].copy_from_slice(item.data);
            pos += item.data.len();
        }

        pos
    }
}

/// Bluetooth LE address (little-endian) used in advertising parameters.
#[derive(Debug, Clone, Copy)]
pub struct BtAddrLe {
    pub addr_type: u8,
    pub addr: [u8; 6],
}

/// Advertising parameter set for legacy advertising.
///
/// This mirrors the Zephyr `struct bt_le_adv_param` layout, trimmed to what
/// the RPC client needs.
pub struct BtLeAdvParam {
    pub id: u8,
    pub sid: u8,
    pub secondary_max_skip: u8,
    pub options: u32,
    pub interval_min: u32,
    pub interval_max: u32,
    pub peer: Option<BtAddrLe>,
}

impl BtLeAdvParam {
    /// Construct a new parameter set.
    pub const fn new(
        id: u8,
        sid: u8,
        secondary_max_skip: u8,
        options: u32,
        interval_min: u32,
        interval_max: u32,
        peer: Option<BtAddrLe>,
    ) -> Self {
        Self {
            id,
            sid,
            secondary_max_skip,
            options,
            interval_min,
            interval_max,
            peer,
        }
    }

    /// Encode this parameter set into a flat byte buffer.
    ///
    /// Layout:
    /// - 1 byte: id
    /// - 1 byte: sid
    /// - 1 byte: secondary_max_skip
    /// - 4 bytes: options (little-endian)
    /// - 4 bytes: interval_min (little-endian)
    /// - 4 bytes: interval_max (little-endian)
    /// - 1 byte: peer_present flag (0 or 1)
    /// - if peer_present:
    ///   - 1 byte: addr_type
    ///   - 6 bytes: addr
    ///
    /// Returns the number of bytes written into `out`.
    pub fn encode_into(&self, out: &mut [u8]) -> usize {
        let mut pos = 0;

        // Minimum size without peer: 1 + 1 + 1 + 4 + 4 + 4 + 1
        if out.len() < 1 + 1 + 1 + 4 + 4 + 4 + 1 {
            return 0;
        }

        out[pos] = self.id;
        pos += 1;
        out[pos] = self.sid;
        pos += 1;
        out[pos] = self.secondary_max_skip;
        pos += 1;

        let opts = self.options.to_le_bytes();
        out[pos..pos + 4].copy_from_slice(&opts);
        pos += 4;

        let imin = self.interval_min.to_le_bytes();
        out[pos..pos + 4].copy_from_slice(&imin);
        pos += 4;

        let imax = self.interval_max.to_le_bytes();
        out[pos..pos + 4].copy_from_slice(&imax);
        pos += 4;

        match &self.peer {
            Some(peer) => {
                // peer_present + addr_type + addr[6]
                if pos + 1 + 1 + 6 > out.len() {
                    return pos;
                }
                out[pos] = 1;
                pos += 1;
                out[pos] = peer.addr_type;
                pos += 1;
                out[pos..pos + 6].copy_from_slice(&peer.addr);
                pos += 6;
            }
            None => {
                if pos + 1 > out.len() {
                    return pos;
                }
                out[pos] = 0;
                pos += 1;
            }
        }

        pos
    }
}

// --------------------------------------------------------------------------
// Convenient advertising parameter constants
// --------------------------------------------------------------------------

// Commonly used "fast connectable" interval range from Zephyr docs.
const BT_GAP_ADV_FAST_INT_MIN_2: u32 = 0x0030;
const BT_GAP_ADV_FAST_INT_MAX_2: u32 = 0x0060;

/// Default legacy connectable advertising parameters, no directed peer.
pub const BT_LE_ADV_PARAM_CONNECTABLE_DEFAULT: BtLeAdvParam = BtLeAdvParam::new(
    0,                         // id
    0,                         // sid
    0,                         // secondary_max_skip
    0,                         // options (no special flags)
    BT_GAP_ADV_FAST_INT_MIN_2, // interval_min
    BT_GAP_ADV_FAST_INT_MAX_2, // interval_max
    None,                      // peer
);

// --------------------------------------------------------------------------
// Tests
// --------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bt_data_encode_single_flags() {
        let flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
        let flags_bytes = [flags];
        let ad = [BtData::flags(&flags_bytes)];

        let mut buf = [0u8; 8];
        let len = BtData::encode_list_into(&ad, &mut buf);

        // type (1) + len (2) + data (1)
        assert_eq!(len, 4);
        assert_eq!(buf[0], 0x01); // Flags AD type
        assert_eq!(u16::from_le_bytes([buf[1], buf[2]]), 1);
        assert_eq!(buf[3], flags);
    }

    #[test]
    fn bt_data_encode_flags_and_name() {
        let flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
        let name = b"MyDevice";
        let flags_bytes = [flags];
        let ad = [BtData::flags(&flags_bytes), BtData::name_complete(name)];

        let mut buf = [0u8; 32];
        let len = BtData::encode_list_into(&ad, &mut buf);

        // First element
        assert_eq!(buf[0], 0x01);
        assert_eq!(u16::from_le_bytes([buf[1], buf[2]]), 1);
        assert_eq!(buf[3], flags);

        // Second element
        let pos = 4;
        assert_eq!(buf[pos], 0x09); // Complete Local Name
        let name_len = u16::from_le_bytes([buf[pos + 1], buf[pos + 2]]) as usize;
        assert_eq!(name_len, name.len());
        assert_eq!(&buf[pos + 3..pos + 3 + name.len()], name);

        // Total length should match computed layout.
        assert_eq!(len, pos + 3 + name.len());
    }

    #[test]
    fn bt_le_adv_param_encode_without_peer() {
        let param = BT_LE_ADV_PARAM_CONNECTABLE_DEFAULT;

        let mut buf = [0u8; 32];
        let len = param.encode_into(&mut buf);

        // 1(id) +1(sid) +1(skip) +4(opts) +4(int_min) +4(int_max) +1(peer_flag)
        assert_eq!(len, 1 + 1 + 1 + 4 + 4 + 4 + 1);

        assert_eq!(buf[0], param.id);
        assert_eq!(buf[1], param.sid);
        assert_eq!(buf[2], param.secondary_max_skip);

        let options = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
        let int_min = u32::from_le_bytes([buf[7], buf[8], buf[9], buf[10]]);
        let int_max = u32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]);

        assert_eq!(options, param.options);
        assert_eq!(int_min, param.interval_min);
        assert_eq!(int_max, param.interval_max);

        assert_eq!(buf[15], 0); // no peer
    }

    #[test]
    fn bt_le_adv_param_encode_with_peer() {
        let peer = BtAddrLe {
            addr_type: 1,
            addr: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
        };
        let param = BtLeAdvParam::new(1, 2, 3, 0xAABB_CCDD, 0x0011_2233, 0x4455_6677, Some(peer));

        let mut buf = [0u8; 64];
        let len = param.encode_into(&mut buf);

        // base (15) + peer_present(1) + addr_type(1) + addr(6) = 23
        assert_eq!(len, 23);

        assert_eq!(buf[0], 1);
        assert_eq!(buf[1], 2);
        assert_eq!(buf[2], 3);

        assert_eq!(
            u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]),
            0xAABB_CCDD
        );
        assert_eq!(
            u32::from_le_bytes([buf[7], buf[8], buf[9], buf[10]]),
            0x0011_2233
        );
        assert_eq!(
            u32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]),
            0x4455_6677
        );

        assert_eq!(buf[15], 1); // peer_present
        assert_eq!(buf[16], 1); // addr_type
        assert_eq!(&buf[17..23], &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
    }
}
