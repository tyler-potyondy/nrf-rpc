//! nRF RPC protocol specification
//!
//! Two processors that communicate with each other using the remote procedure call
//! (nRF RPC) library follow the nRF RPC protocol. The nRF RPC protocol specifies
//! the binary format and rules for constructing packets that are exchanged within an
//! nRF RPC communication.
//!
//! The nRF RPC packets that are constructed by the nRF RPC core library are relayed
//! to the selected transport layer, where they can be additionally encoded to ensure
//! a reliable delivery of the packet to the other processor using the selected medium.
//! The nRF RPC transport's specification is outside the scope of this document.
//!
//! Source: https://github.com/nrfconnect/sdk-nrfxlib/blob/6204e5fcdac22b4309c72b990857fcc28d8c3095/nrf_rpc/doc/protocol_specification.rst

/// nRF RPC packet format
///
/// Each nRF RPC packet consists of a 5-byte header and an optional, variable-length payload:
///   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///   |0                              |1                              |
///   +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///   |0  |1  |2  |3  |4  |5  |6  |7  |0  |1  |2  |3  |4  |5  |6  |7  |
///   +===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+
///   | Type [\| Source Context ID]   | Command ID                    |
///   +-------------------------------+-------------------------------+
///   | Destination Context ID        | Source Group ID               |
///   +-------------------------------+-------------------------------+
///   | Destination Group ID          | [Payload...]                  |
///   +-------------------------------+-------------------------------+
///   |                             [...]                             |
///   +---------------------------------------------------------------+
/// [| Source Context ID]: 8 bits
///
/// The packet type determines the function of the packet and it can be one of the following values:
///   - 0x00: event
///   - 0x01: response
///   - 0x02: event acknowledgment
///   - 0x03: error report
///   - 0x04: initialization packet
///   - 0x80: command
///
///  If the packet type is 0x80 (command), this field is additionally bitwise ORed with
///  the source context ID.
///
///  The source context ID is a numeric identifier of the conversation to which the packet
///  is associated, chosen by the packet sender.
///
///  The source context ID is a feature of the nRF RPC protocol that facilitates concurrent
///  conversations. When two threads on the local processor want to start an nRF RPC
///  conversation at the same time, they shall use distinct source context IDs when constructing
///  a packet to the remote processor. The remote processor is then obliged to use the source
///  context ID as the destination context ID in the response packet. This ensures that responses
///  and any packets that follow within each conversation are correctly routed to the initiating thread.
///
///  The exact source context ID allocation pattern is implementation-defined, meaning that when the
///  packet sender initiates a new conversation or responds to the initiating packet, it is free to
///  allocate any unused source context ID for the new conversation.
/// Payload: variable length
///
/// The payload format depends on the packet type:
///   - event acknowledgment: the payload is empty.
///   - error report: the payload is a 32-bit integer representing an error code, in
///     little-endian byte order.
///   - initialization packet: the payload has the following format:
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0                              |1                              |
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0  |1  |2  |3  |4  |5  |6  |7  |0  |1  |2  |3  |4  |5  |6  |7  |
///     +===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+
///     | Max Version   | Min Version   | Group name....                |
///     +---------------+---------------+-------------------------------+
///     |                              ...                              |
///     +---------------------------------------------------------------+
///
/// The Min Version and Max Version fields indicate the minimum and maximum
/// version of the nRF RPC protocol supported by the sender. The Group name
/// field has a variable length and contains the string identifier of the nRF
/// RPC group to which this packet is associated with, without the null terminator.
/*
  * **event**, **response**, **command** - the payload contains remote procedure call arguments or return values, represented in an implementation-defined format.

    If the nRF RPC protocol is used together with the CBOR encoding, then the arguments and return values are represented as a sequence of CBOR data items, terminated by the null data item (``0xf6``).

    For example, if a packet is an nRF RPC command that represents the C function call ``foo(100, "bar")``, the packet might look as follows:

    .. code-block:: none

       80 01 ff 00 00 18 64 63 62 61 72 f6

       80: Command | Source Context ID (0)
       01: Command ID (1)
       ff: Destination Context ID (unknown)
       00: Source Group ID (0)
       00: Destination Group ID (0)
       18 64: CBOR unsigned int (100)
       63 62 61 72: CBOR text string ("bar")
       f6: CBOR null
*/
use minicbor::encode::Encoder;

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
