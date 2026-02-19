/*
/// Builder for constructing NRF RPC packets
///
/// Note: This is exposed for testing purposes only. Use the `Ble` struct for normal usage.
#[doc(hidden)]
pub struct PacketBuilder<const N: usize> {
    buffer: [u8; N],
    pos: usize,
}

impl<const N: usize> PacketBuilder<N> {
    pub fn new() -> Self {
        Self {
            buffer: [0u8; N],
            pos: 0,
        }
    }

    /// Build an initialization packet
    ///
    /// Format: 0x04 | 0x00 | 0xFF | src_grp_id | 0xFF | 0x00 (version) | group_name
    pub fn init(mut self, src_group_id: u8, group_name: &str) -> Self {
        self.buffer[0] = PacketType::Init as u8;
        self.buffer[1] = 0x00; // Command ID unused for init
        self.buffer[2] = 0xFF; // Destination context unknown
        self.buffer[3] = src_group_id;
        self.buffer[4] = 0xFF; // Destination group unknown
        self.buffer[5] = 0x00; // Version
        self.pos = 6;

        // Append group name bytes
        let name_bytes = group_name.as_bytes();
        self.buffer[self.pos..self.pos + name_bytes.len()].copy_from_slice(name_bytes);
        self.pos += name_bytes.len();

        self
    }

    /// Build a command packet header
    ///
    /// Format: 0x80 | src_ctx_id | cmd_id | dst_ctx_id | src_grp_id | dst_grp_id
    pub fn command(
        mut self,
        src_ctx_id: u8,
        cmd_id: u8,
        dst_ctx_id: u8,
        src_grp_id: u8,
        dst_grp_id: u8,
    ) -> Self {
        self.buffer[0] = PacketType::Command as u8 | src_ctx_id;
        self.buffer[1] = cmd_id;
        self.buffer[2] = dst_ctx_id;
        self.buffer[3] = src_grp_id;
        self.buffer[4] = dst_grp_id;
        self.pos = 5;
        self
    }

    /// Encode an unsigned integer in CBOR format to the payload
    pub fn cbor_uint(mut self, value: u64) -> Result<Self, CborError> {
        let mut writer = SliceWriter::new(&mut self.buffer[self.pos..]);
        let mut encoder = Encoder::new(&mut writer);
        encoder.u64(value)?;
        self.pos += writer.pos();
        Ok(self)
    }

    /// Encode a signed integer in CBOR format to the payload
    pub fn cbor_int(mut self, value: i64) -> Result<Self, CborError> {
        let mut writer = SliceWriter::new(&mut self.buffer[self.pos..]);
        let mut encoder = Encoder::new(&mut writer);
        encoder.i64(value)?;
        self.pos += writer.pos();
        Ok(self)
    }

    /// Encode bytes in CBOR format to the payload
    pub fn cbor_bytes(mut self, bytes: &[u8]) -> Result<Self, CborError> {
        let mut writer = SliceWriter::new(&mut self.buffer[self.pos..]);
        let mut encoder = Encoder::new(&mut writer);
        encoder.bytes(bytes)?;
        self.pos += writer.pos();
        Ok(self)
    }

    /// Encode a string in CBOR format to the payload
    pub fn cbor_str(mut self, s: &str) -> Result<Self, CborError> {
        let mut writer = SliceWriter::new(&mut self.buffer[self.pos..]);
        let mut encoder = Encoder::new(&mut writer);
        encoder.str(s)?;
        self.pos += writer.pos();
        Ok(self)
    }

    /// Encode CBOR null (0xF6) - used as packet terminator
    pub fn cbor_null(mut self) -> Result<Self, CborError> {
        let mut writer = SliceWriter::new(&mut self.buffer[self.pos..]);
        let mut encoder = Encoder::new(&mut writer);
        encoder.null()?;
        self.pos += writer.pos();
        Ok(self)
    }

    /// Get the packet bytes as a slice
    pub fn as_slice(&self) -> &[u8] {
        &self.buffer[..self.pos]
    }

    /// Get the length of the packet
    pub fn len(&self) -> usize {
        self.pos
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cbor_uint_encoding() {
        // Test encoding uint(28) - should be 0x18 0x1C
        let packet = PacketBuilder::<32>::new().cbor_uint(28).unwrap();
        assert_eq!(packet.as_slice(), &[0x18, 0x1C]);

        // Test encoding uint(160) - should be 0x18 0xA0
        let packet2 = PacketBuilder::<32>::new().cbor_uint(160).unwrap();
        assert_eq!(packet2.as_slice(), &[0x18, 0xA0]);

        // Test small uint (0-23) - encoded directly
        let packet3 = PacketBuilder::<32>::new().cbor_uint(3).unwrap();
        assert_eq!(packet3.as_slice(), &[0x03]);
    }

    #[test]
    fn test_init_packet() {
        // Build init packet for "bt_rpc"
        let packet = PacketBuilder::<64>::new().init(0x00, "bt_rpc");

        let expected = &[
            0x04, 0x00, 0xFF, 0x00, 0xFF, 0x00, b'b', b't', b'_', b'r', b'p', b'c',
        ];
        assert_eq!(packet.as_slice(), expected);
    }

    #[test]
    fn test_bt_enable_packet() {
        // Build bt_enable command packet matching raw_rpc trace
        let packet = PacketBuilder::<64>::new()
            .command(0x00, 0x00, 0xFF, 0x00, 0x00)
            .cbor_uint(28)
            .unwrap() // scratchpad_size
            .cbor_uint(28)
            .unwrap() // callback_slot
            .cbor_null()
            .unwrap(); // terminator

        let expected = &[
            0x80, 0x00, 0xFF, 0x00, 0x00, 0x18, 0x1C, // uint(28)
            0x18, 0x1C, // uint(28)
            0xF6,
        ]; // null
        assert_eq!(packet.as_slice(), expected);
    }
}

use minicbor::encode::Encoder;


/// Packet type identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PacketType {
    Event = 0x00,
    Response = 0x01,
    EventAck = 0x02,
    ErrorReport = 0x03,
    Init = 0x04,
    Command = 0x80,
}
*/
use minicbor::Encoder;

pub struct CBorPayload<'a>(&'a [u8]);
impl<'a> Into<&'a [u8]> for CBorPayload<'a> {
    fn into(self) -> &'a [u8] {
        self.0
    }
}

pub struct CborPayloadBuilder<'a> {
    buffer: &'a mut [u8],
    pos: usize,
}

impl<'a> CborPayloadBuilder<'a> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, pos: 0 }
    }

    fn encode<F>(&mut self, encode_fn: F) -> Result<&mut Self, CborError>
    where
        F: FnOnce(&mut Encoder<SliceWriter>) -> Result<(), CborError>,
    {
        let writer = SliceWriter::new(&mut self.buffer[self.pos..]); // No need for mut
        let mut encoder = Encoder::new(writer); // Declare encoder as mutable
        encode_fn(&mut encoder)?; // Ensure this returns Result<(), CborError>
        self.pos += encoder.writer().pos(); // Use encoder to get position
        Ok(self)
    }

    pub fn encode_uint(mut self, value: u64) -> Result<Self, CborError> {
        self.encode(|encoder| {
            encoder.u64(value)?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?;
        Ok(self)
    }

    pub fn cbor_int(mut self, value: i64) -> Result<Self, CborError> {
        self.encode(|encoder| {
            encoder.i64(value)?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?;
        Ok(self)
    }

    pub fn cbor_bytes(mut self, bytes: &[u8]) -> Result<Self, CborError> {
        self.encode(|encoder| {
            encoder.bytes(bytes)?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?;
        Ok(self)
    }

    pub fn cbor_str(mut self, s: &str) -> Result<Self, CborError> {
        self.encode(|encoder| {
            encoder.str(s)?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?;
        Ok(self)
    }

    pub fn cbor_null(mut self) -> Result<Self, CborError> {
        self.encode(|encoder| {
            encoder.null()?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?;
        Ok(self)
    }

    pub fn build(mut self) -> Result<CBorPayload<'a>, CborError> {
        self.encode(|encoder| {
            encoder.null()?;
            Ok(()) // Ensure closure returns Result<(), CborError>
        })?; // Add CBOR null terminator (0xF6)

        let new_buffer = &self.buffer[..self.pos];
        Ok(CBorPayload(new_buffer))
    }
}

/// A writer that writes to a mutable slice and tracks position
struct SliceWriter<'a> {
    slice: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    fn new(slice: &'a mut [u8]) -> Self {
        Self { slice, pos: 0 }
    }

    fn pos(&self) -> usize {
        self.pos
    }
}

impl<'a> minicbor::encode::Write for SliceWriter<'a> {
    type Error = CborError;

    fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        if self.pos + buf.len() > self.slice.len() {
            return Err(CborError::BufferTooSmall);
        }
        self.slice[self.pos..self.pos + buf.len()].copy_from_slice(buf);
        self.pos += buf.len();
        Ok(())
    }
}

// todo
/// CBOR encoding error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CborError {
    BufferTooSmall,
    EncodingError,
}

impl core::fmt::Display for CborError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CborError::BufferTooSmall => write!(f, "CBOR buffer too small"),
            CborError::EncodingError => write!(f, "CBOR encoding error"),
        }
    }
}

impl From<minicbor::encode::Error<CborError>> for CborError {
    fn from(_: minicbor::encode::Error<CborError>) -> Self {
        CborError::EncodingError
    }
}
