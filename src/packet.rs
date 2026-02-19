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

use crate::cbor_encoding::CBorPayload;
use core::marker::PhantomData;

const EVENT_PACKET_TYPE: u8 = 0x00;
const RESPONSE_PACKET_TYPE: u8 = 0x01;
const EVENT_ACK_PACKET_TYPE: u8 = 0x02;
const ERROR_REPORT_PACKET_TYPE: u8 = 0x03;
const INIT_PACKET_TYPE: u8 = 0x04;
const COMMAND_PACKET_TYPE_BASE: u8 = 0x80;

/// Command ID field value indicating that the field is unused
/// (e.g., for response and init packets).
const COMMAND_ID_FIELD_UNUSED: u8 = 0xFF;

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
///
/// Type [| Source Context ID]: 8 bits
/// ==================================
/// The packet type determines the function of the packet and it can be
/// one of the following values:
///   - 0x00: event
///   - 0x01: response
///   - 0x02: event acknowledgment
///   - 0x03: error report
///   - 0x04: initialization packet
///   - 0x80: command
///
/// If the packet type is 0x80 (command), this field is additionally bitwise
/// ORed with the source context ID.
///
/// The source context ID is a numeric identifier of the conversation to which
/// the packet is associated, chosen by the packet sender.
///
/// The source context ID is a feature of the nRF RPC protocol that facilitates
/// concurrent conversations. When two threads on the local processor want to
/// start an nRF RPC conversation at the same time, they shall use distinct source
/// context IDs when constructing a packet to the remote processor. The remote
/// processor is then obliged to use the source context ID as the destination context
/// ID in the response packet. This ensures that responses and any packets that
/// follow within each conversation are correctly routed to the initiating thread.
///
/// The exact source context ID allocation pattern is implementation-defined, meaning
/// that when the packet sender initiates a new conversation or responds to the
/// initiating packet, it is free to allocate any unused source context ID for the new
/// conversation.
///
/// Command ID: 8 bits
/// ==================
/// Identifies an individual command or event within an nRF RPC group.
///
/// If the packet is a response or an initialization packet, this field has no meaning
/// and shall be set to 0xff.
///
/// Destination Context ID: 8 bits
/// ==============================
/// A numeric identifier of the conversation to which the packet is associated,
/// chosen by the packet receiver.
///
/// In a packet that starts a new conversation, this field shall be assigned
/// the value 0xff (indicating it is unknown). In all subsequent packets within
/// the conversation, the sender of the packet shall carry over the source context
/// ID that was included in the last packet received from the peer.
///
/// Source Group ID: 8 bits
/// =======================
/// A numeric identifier of the nRF RPC group associated with the packet, chosen
/// by the packet sender.
///
/// Each processor that uses the nRF RPC protocol chooses unique numeric identifiers
/// for all nRF RPC groups that it supports. During the nRF RPC protocol initialization,
/// it then communicates its own mapping of the pre-shared string group identifiers to
/// these unique numeric identifiers.
///
/// Destination Group ID: 8 bits
/// ============================
/// A numeric identifier of the nRF RPC group associated with the packet, chosen by
/// the packet receiver.
///
/// The sender learns this identifier by receiving an initialization packet from the
/// peer during the nRF RPC protocol initialization.
///
/// Payload: variable length
/// ========================
/// The payload format depends on the packet type:
///   - event acknowledgment: the payload is empty.
///   - error report: the payload is a 32-bit integer representing an error code, in
///     little-endian byte order.
///   - initialization packet: the payload has the following format:
///     ```text
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0                              |1                              |
///     +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
///     |0  |1  |2  |3  |4  |5  |6  |7  |0  |1  |2  |3  |4  |5  |6  |7  |
///     +===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+===+
///     | Max Version   | Min Version   | Group name....                |
///     +---------------+---------------+-------------------------------+
///     |                              ...                              |
///     +---------------------------------------------------------------+
///     ```
///     The Min Version and Max Version fields indicate the minimum and maximum
///     version of the nRF RPC protocol supported by the sender. The Group name
///     field has a variable length and contains the string identifier of the nRF
///     RPC group to which this packet is associated with, without the null terminator.
///   - event, response, command: the payload contains remote procedure call
///     arguments or return values, represented in an implementation-defined format. If
///     the nRF RPC protocol is used together with the CBOR encoding, then the
///     arguments and return values are represented as a sequence of CBOR data items,
///     terminated by the null data item (0xf6).
///
///     For example, if a packet is an nRF RPC command that represents the C function
///     call `foo(100, "bar")`, the packet might look as follows:
///       80 01 ff 00 00 18 64 63 62 61 72 f6
///
///       80: Command | Source Context ID (0)
///       01: Command ID (1)
///       ff: Destination Context ID (unknown)
///       00: Source Group ID (0)
///       00: Destination Group ID (0)
///       18 64: CBOR unsigned int (100)
///       63 62 61 72: CBOR text string ("bar")
///       f6: CBOR null
pub struct NrfRpcPacket<'a, T: NrfRpcPacketType> {
    src_context_id: Option<u8>, // Only set for command packets
    dst_context_id: u8,
    src_group_id: u8,
    dst_group_id: u8,
    payload: &'a [u8],
    associated_packet_type: PhantomData<T>,
}

impl<'a, C: CommandId> NrfRpcPacket<'a, Event<C>> {
    fn new(
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        cbor_encoded_payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a, C: CommandId> NrfRpcPacket<'a, Response<C>> {
    fn new(
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        cbor_encoded_payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a, C: CommandId> NrfRpcPacket<'a, EventAck<C>> {
    fn new(dst_context_id: u8, src_group_id: u8, dst_group_id: u8) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: &[],
            associated_packet_type: PhantomData,
        }
    }
}

pub struct NrfRpcErrorCode<'a>(&'a mut [u8; 4]);
impl<'a> NrfRpcErrorCode<'a> {
    pub fn new(buffer: &'a mut [u8; 4]) -> Self {
        Self(buffer)
    }

    pub fn set_error_code(&mut self, error_code: u32) {
        self.0.copy_from_slice(&error_code.to_le_bytes());
    }
}

impl<'a> Into<&'a [u8]> for NrfRpcErrorCode<'a> {
    fn into(self) -> &'a [u8] {
        &self.0[..]
    }
}

// ErrorReport packets have a fixed payload size of 4 bytes (32-bit error code)
impl<'a> NrfRpcPacket<'a, ErrorReport> {
    // Error report: the payload is a 32-bit integer representing an error code, in
    // little-endian byte order.
    fn new(
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        error_code: NrfRpcErrorCode<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: error_code.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a, C: CommandId> NrfRpcPacket<'a, Command<C>> {
    pub fn new(
        src_context_id: u8,
        dst_context_id: u8,
        src_group_id: u8,
        dst_group_id: u8,
        payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: Some(src_context_id),
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload: payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

struct InitPacketPayload<'a, const N: usize> {
    data: &'a mut [u8; N],
}

impl<'a, const N: usize> InitPacketPayload<'a, N> {
    pub fn new(buffer: &'a mut [u8; N]) -> Self {
        Self { data: buffer }
    }

    pub fn set_version(&mut self, max_version: u8, min_version: u8) {
        self.data[0] = (max_version & 0x0F) | ((min_version & 0x0F) << 4);
    }

    pub fn set_group_name(&mut self, group_name: &str) -> Result<(), ()> {
        let name_bytes = group_name.as_bytes();
        if name_bytes.len() > self.data.len() - 1 {
            return Err(()); // Not enough space for group name
        }

        // (todo) this panic path should not be reached due to above check,
        // but it would be better to statically guarantee this.
        self.data[1..1 + name_bytes.len()].copy_from_slice(name_bytes);
        Ok(())
    }
}

impl<'a, const N: usize> Into<&'a [u8]> for InitPacketPayload<'a, N> {
    fn into(self) -> &'a [u8] {
        &self.data[..]
    }
}

impl<'a> NrfRpcPacket<'a, Init> {
    // (todo) use flux or nightly feature for const generic arith
    // to add this to the type system (instead of just payload array here).
    // Max Version: bits 0-3 (byte 0)
    // Min Version: bits 4-7 (byte 0)
    // Group name: byte 1 to N
    fn new<const N: usize>(
        src_group_id: u8,
        dst_group_id: u8,
        init_payload: InitPacketPayload<'a, N>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id: 0xFF, // Unknown destination context ID for init packets
            src_group_id,
            dst_group_id,
            payload: init_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a, P: NrfRpcPacketType> NrfRpcPacket<'a, P> {
    fn form_header(&self) -> [u8; 5] {
        let type_byte = P::TypeField | self.src_context_id.unwrap_or(0);
        let command_id_byte = P::CommandIdField;
        [
            type_byte,
            command_id_byte,
            self.dst_context_id,
            self.src_group_id,
            self.dst_group_id,
        ]
    }

    /// Provided an RpcTransportBuffer, copy the formed nrf rpc packet into the
    /// buffer. Returns Result<(), ErrorCode>.
    pub fn write_into<const N: usize>(
        &self,
        buf: &mut crate::RpcTransportBuffer<N>,
    ) -> Result<(), ()> {
        // (todo) error code update to not be `()`
        // (todo) this requires copying. I would rather this be zero copy,
        // but alas...we could pretty easily do some unsafe shenanigans
        // to avoid copying, but for now we will just copy.
        if buf.remaining_len() < self.payload.len() + 5 {
            return Err(()); // Buffer too small
        }

        // (todo) it would be nice to avoid this panic path.
        let header = self.form_header();
        buf.write_into_or_err(&header)?;
        buf.write_into_or_err(self.payload)?;
        Ok(())
    }
}

pub trait CommandId {
    const COMMAND_ID: u8;
}

pub trait NrfRpcPacketType {
    const TypeField: u8;
    const CommandIdField: u8;
}

pub struct Event<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Event<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = 0x00;
}

pub struct Response<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Response<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = RESPONSE_PACKET_TYPE;
}

pub struct EventAck<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for EventAck<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = EVENT_ACK_PACKET_TYPE;
}

pub struct ErrorReport;
impl NrfRpcPacketType for ErrorReport {
    const CommandIdField: u8 = unimplemented!();
    const TypeField: u8 = ERROR_REPORT_PACKET_TYPE;
}

pub struct Init;
impl NrfRpcPacketType for Init {
    const CommandIdField: u8 = COMMAND_ID_FIELD_UNUSED;
    const TypeField: u8 = INIT_PACKET_TYPE;
}

pub struct Command<C: CommandId>(PhantomData<C>);
impl<C: CommandId> NrfRpcPacketType for Command<C> {
    const CommandIdField: u8 = C::COMMAND_ID;
    const TypeField: u8 = COMMAND_PACKET_TYPE_BASE;
}
