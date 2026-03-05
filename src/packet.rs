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
/// and shall be set to 0xff. **NOTE: This is copied from the docs, but it appears that
/// in practice, the Command ID field is set to 0x00 for resp/init packets.**
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
    src_context_id: Option<SrcContextId>, // Only set for command packets
    dst_context_id: DestContextId,
    command_id: CommandId,
    src_group_id: SrcGroupId,
    dst_group_id: DstGroupId,
    payload: &'a [u8],
    associated_packet_type: PhantomData<T>,
}

pub struct DestContextId(u8);

impl TryFrom<u8> for DestContextId {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl Into<u8> for DestContextId {
    fn into(self) -> u8 {
        self.0
    }
}

pub struct SrcGroupId(u8);

impl TryFrom<u8> for SrcGroupId {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl Into<u8> for SrcGroupId {
    fn into(self) -> u8 {
        self.0
    }
}

#[derive(Copy, Clone)]
pub struct DstGroupId(u8);

impl TryFrom<u8> for DstGroupId {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl Into<u8> for DstGroupId {
    fn into(self) -> u8 {
        self.0
    }
}

pub struct CommandId(u8);

impl TryFrom<u8> for CommandId {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl Into<u8> for CommandId {
    fn into(self) -> u8 {
        self.0
    }
}

pub struct SrcContextId(u8);
const COMMAND_ID_FIELD_UNUSED: CommandId = CommandId(0x0);

impl TryFrom<u8> for SrcContextId {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl Into<u8> for SrcContextId {
    fn into(self) -> u8 {
        self.0
    }
}

impl<'a> NrfRpcPacket<'a, Event> {
    pub fn new(
        dst_context_id: DestContextId,
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        command_id: CommandId,
        cbor_encoded_payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            command_id,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a> NrfRpcPacket<'a, Response> {
    pub fn new(
        dst_context_id: DestContextId,
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        cbor_encoded_payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            command_id: COMMAND_ID_FIELD_UNUSED,
            src_group_id,
            dst_group_id,
            payload: cbor_encoded_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a> NrfRpcPacket<'a, EventAck> {
    pub fn new(
        dst_context_id: DestContextId,
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        command_id: CommandId,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id: dst_context_id,
            command_id,
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
    pub fn new(
        dst_context_id: DestContextId,
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        command_id: CommandId,
        error_code: NrfRpcErrorCode<'a>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id,
            command_id,
            src_group_id,
            dst_group_id,
            payload: error_code.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a> NrfRpcPacket<'a, Command> {
    pub fn new(
        src_context_id: SrcContextId,
        dst_context_id: DestContextId,
        command_id: CommandId,
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        payload: CBorPayload<'a>,
    ) -> Self {
        Self {
            src_context_id: Some(src_context_id),
            dst_context_id,
            command_id,
            src_group_id,
            dst_group_id,
            payload: payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

pub struct InitPacketPayload<'a, const N: usize> {
    data: &'a mut [u8; N],
    pos: usize,
}

pub struct MaxVersion(u8);
impl MaxVersion {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }
}

impl From<MaxVersion> for u8 {
    fn from(value: MaxVersion) -> Self {
        value.0
    }
}

impl TryFrom<u8> for MaxVersion {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

pub struct MinVersion(u8);
impl MinVersion {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }
}

impl From<MinVersion> for u8 {
    fn from(value: MinVersion) -> Self {
        value.0
    }
}

impl TryFrom<u8> for MinVersion {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(Self(value))
    }
}

impl<'a, const N: usize> InitPacketPayload<'a, N> {
    pub fn new(
        buffer: &'a mut [u8; N],
        max_version: MaxVersion,
        min_version: MinVersion,
        group_name: &str,
    ) -> Result<Self, ()> {
        if group_name.as_bytes().len() > N - 1 {
            return Err(()); // Not enough space for version and group name
        }

        buffer[0] = (u8::from(max_version) & 0x0F) | ((u8::from(min_version) & 0x0F) << 4);
        buffer[1..1 + group_name.as_bytes().len()].copy_from_slice(group_name.as_bytes());
        Ok(Self {
            data: buffer,
            pos: 1 + group_name.as_bytes().len(),
        })
    }
}

impl<'a, const N: usize> From<InitPacketPayload<'a, N>> for &'a [u8] {
    fn from(value: InitPacketPayload<'a, N>) -> Self {
        &value.data[..value.pos]
    }
}

impl<'a> NrfRpcPacket<'a, Init> {
    // (todo) use flux or nightly feature for const generic arith
    // to add this to the type system (instead of just payload array here).
    // Max Version: bits 0-3 (byte 0)
    // Min Version: bits 4-7 (byte 0)
    // Group name: byte 1 to N
    pub fn new<const N: usize>(
        src_group_id: SrcGroupId,
        dst_group_id: DstGroupId,
        init_payload: InitPacketPayload<'a, N>,
    ) -> Self {
        Self {
            src_context_id: None,
            dst_context_id: DestContextId(0xFF), // Unknown destination context ID for init packets
            command_id: COMMAND_ID_FIELD_UNUSED,
            src_group_id,
            dst_group_id,
            payload: init_payload.into(),
            associated_packet_type: PhantomData,
        }
    }
}

impl<'a, P: NrfRpcPacketType> NrfRpcPacket<'a, P> {
    fn form_packet(self) -> ([u8; 5], &'a [u8]) {
        let type_byte = P::PACKET_TYPE as u8
            | <SrcContextId as Into<u8>>::into(self.src_context_id.unwrap_or(SrcContextId(0)));
        let command_id_byte = self.command_id.into();
        let header = [
            type_byte,
            command_id_byte,
            self.dst_context_id.into(),
            self.src_group_id.into(),
            self.dst_group_id.into(),
        ];
        (header, self.payload)
    }

    /// Provided an RpcTransportBuffer, copy the formed nrf rpc packet into the
    /// buffer. Returns Result<(), ErrorCode>.
    pub fn write_into<const N: usize, T: crate::transport::RpcTransportBuffer<'a, N>>(
        self,
        buf: &mut T,
    ) -> Result<(), ()> {
        // (todo) error code update to not be `()`
        // (todo) this requires copying. I would rather this be zero copy,
        // but alas...we could pretty easily do some unsafe shenanigans
        // to avoid copying, but for now we will just copy.

        // (todo) it would be nice to avoid this panic path.
        let (header, payload) = self.form_packet();
        buf.write_slice_into_or_err(&header)?;
        buf.write_slice_into_or_err(payload)?;
        Ok(())
    }
}

pub trait NrfRpcPacketType {
    const PACKET_TYPE: TypeField;
}

pub enum TypeField {
    Event = 0x00,
    Response = 0x01,
    EventAck = 0x02,
    ErrorReport = 0x03,
    Init = 0x04,
    Command = 0x80,
}

impl TryFrom<u8> for TypeField {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        unimplemented!()
    }
}

pub struct Event;
impl NrfRpcPacketType for Event {
    const PACKET_TYPE: TypeField = TypeField::Event;
}

pub struct Response;
impl NrfRpcPacketType for Response {
    const PACKET_TYPE: TypeField = TypeField::Response;
}

pub struct EventAck;
impl NrfRpcPacketType for EventAck {
    const PACKET_TYPE: TypeField = TypeField::EventAck;
}

pub struct ErrorReport;
impl NrfRpcPacketType for ErrorReport {
    const PACKET_TYPE: TypeField = TypeField::ErrorReport;
}

pub struct Init;
impl NrfRpcPacketType for Init {
    const PACKET_TYPE: TypeField = TypeField::Init;
}

pub struct Command;
impl NrfRpcPacketType for Command {
    const PACKET_TYPE: TypeField = TypeField::Command;
}
