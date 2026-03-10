use crate::packet::{
    Command, CommandId, DestContextId, DstGroupId, ErrorReport, Event, EventAck, Init,
    InitPacketPayload, MaxVersion, MinVersion, NrfRpcErrorCode, NrfRpcPacket, Response,
    SrcContextId, SrcGroupId, TypeField,
};

use crate::cbor_encoding::CBorPayload;

pub struct ParsedNrfRpcPacket<'a> {
    pub packet_type: TypeField,
    pub src_context_id: SrcContextId,
    pub command_id: CommandId,
    pub dst_context_id: DestContextId,
    pub src_group_id: SrcGroupId,
    pub dst_group_id: DstGroupId,
    pub payload: ParsedPayload<'a>,
}

pub enum ParsedPayload<'a> {
    Cbor(CBorPayload<'a>),
    ErrorCode(u32),
    InitPacketPayload(MinVersion, MaxVersion, &'a str),
}

enum RawPayload<'a> {
    Cbor(&'a mut [u8]),
    ErrorCode(&'a mut [u8]),
    InitPacketPayload(&'a mut [u8]),
}

impl<'a> TryFrom<RawPayload<'a>> for ParsedPayload<'a> {
    type Error = ();
    fn try_from(value: RawPayload<'a>) -> Result<Self, Self::Error> {
        match value {
            RawPayload::Cbor(payload) => Ok(ParsedPayload::Cbor(
                CBorPayload::try_from(payload).map_err(|_| ())?,
            )),
            RawPayload::ErrorCode(payload) => Ok(ParsedPayload::ErrorCode(u32::from_le_bytes(
                payload.try_into().map_err(|_| ())?,
            ))),
            RawPayload::InitPacketPayload(payload) => Ok(ParsedPayload::InitPacketPayload(
                MinVersion::try_from(payload[0])?,
                MaxVersion::try_from(payload[1])?,
                core::str::from_utf8(&payload[2..]).map_err(|_| ())?,
            )),
        }
    }
}

impl<'a> TryFrom<&'a mut [u8]> for ParsedNrfRpcPacket<'a> {
    type Error = ();
    fn try_from(value: &'a mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < 5 {
            return Err(()); // Buffer too small to contain header
        }

        let (packet_type, src_context_id): (TypeField, SrcContextId) =
            (value[0].try_into()?, value[0].try_into()?);

        let command_id: CommandId = value[1].try_into()?;
        let dst_context_id: DestContextId = value[2].try_into()?;
        let src_group_id: SrcGroupId = value[3].try_into()?;
        let dst_group_id: DstGroupId = value[4].try_into()?;

        // (TODO) remove panic path here.
        let raw_payload: RawPayload = match packet_type {
            TypeField::ErrorReport => RawPayload::ErrorCode(&mut value[5..]),
            TypeField::Init => RawPayload::InitPacketPayload(&mut value[5..]),
            TypeField::Event | TypeField::Response | TypeField::EventAck | TypeField::Command => {
                RawPayload::Cbor(&mut value[5..])
            }
        };

        let payload: ParsedPayload = raw_payload.try_into()?;

        Ok(ParsedNrfRpcPacket {
            packet_type,
            src_context_id,
            command_id,
            dst_context_id,
            src_group_id,
            dst_group_id,
            payload,
        })
    }
}
