//! Transport layer traits for NRF RPC
//!
//! This module defines the transport abstraction that users must implement
//! to provide byte-level communication for the RPC protocol.

// async_fn_in_trait is expected for embedded no_std usage
#![allow(async_fn_in_trait)]

use core::fmt;

use crate::{decoding::ParsedNrfRpcPacket, packet::NrfRpcPacket};

/// Error trait for transport implementations
pub trait TransportError: fmt::Debug {}

/// Async transport for sending/receiving raw bytes over UART
///
/// Users implement this trait for their specific UART hardware (e.g., Embassy UART).
/// The transport only needs to provide raw byte read/write - the NRF RPC library
/// handles all framing and packet delimiting.
///
/// # Example
///
/// ```ignore
/// use embassy_nrf::uarte::{Uarte, Instance};
/// use embedded_io_async::{Read, Write};
///
/// struct EmbassyUartTransport<'d, T: Instance> {
///     uart: Uarte<'d, T>,
/// }
///
/// impl<'d, T: Instance> AsyncTransport for EmbassyUartTransport<'d, T> {
///     type Error = UartError;
///     
///     async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
///         self.uart.write(data).await.map_err(|_| UartError::WriteFailed)
///     }
///     
///     async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
///         self.uart.read(buffer).await.map_err(|_| UartError::ReadFailed)
///     }
/// }
/// ```
pub trait AsyncTransport {
    /// Error type for this transport
    type Error: TransportError;
    type TxTransportPacket<'a>: RpcTxTransportPacket<'a>;
    type RxTransportPacket<'a>: RpcRxTransportPacket<'a>;

    /// Write bytes to the transport
    ///
    /// Should block until all bytes are written or an error occurs.
    async fn write(&mut self, data: &mut [u8]) -> Result<usize, Self::Error>;

    /// Read bytes from the transport into the provided buffer
    ///
    /// Returns the number of bytes read. May return fewer bytes than
    /// the buffer size if data is not immediately available.
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;
}

#[derive(Debug)]
pub struct TransportBuffer<'a> {
    buffer: &'a mut [u8],
    start_pos: usize,
    end_pos: usize,
}

/// From a mutable buffer to a Transport Buffer
impl<'a> From<&'a mut [u8]> for TransportBuffer<'a> {
    fn from(value: &'a mut [u8]) -> Self {
        let len = value.len();
        Self {
            buffer: value,
            start_pos: 0,
            end_pos: len,
        }
    }
}

impl<'a> From<&'a TransportBuffer<'a>> for &'a [u8] {
    fn from(value: &'a TransportBuffer<'a>) -> Self {
        if value.start_pos >= value.end_pos {
            panic!(
                "Start position is greater than end position {:02x?}, start_pos: {}, end_pos: {}",
                value.buffer, value.start_pos, value.end_pos
            );
        }
        &value.buffer[value.start_pos..value.end_pos]
    }
}
impl<'a> From<TransportBuffer<'a>> for &'a mut [u8] {
    fn from(value: TransportBuffer<'a>) -> Self {
        if value.start_pos >= value.end_pos {
            panic!(
                "Start position is greater than end position {:02x?}, start_pos: {}, end_pos: {}",
                value.buffer, value.start_pos, value.end_pos
            );
        }
        &mut value.buffer[value.start_pos..value.end_pos]
    }
}

impl<'a> TransportBuffer<'a> {
    /// Creates a new RpcTransportBuffer from a mutable buffer.
    ///
    /// The buffer is assumed to be uninitialized.
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self {
            buffer,
            start_pos: 0,
            end_pos: 0,
        }
    }
}

impl<'a> TransportBuffer<'a> {
    pub fn remaining_len(&self) -> usize {
        self.buffer.len() - self.end_pos
    }

    pub fn full_len(&self) -> usize {
        self.buffer.len()
    }

    pub fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.end_pos + data.len() > self.buffer.len() {
            return Err(());
        }
        self.buffer[self.end_pos..self.end_pos + data.len()].copy_from_slice(data);
        self.end_pos += data.len();
        Ok(())
    }

    pub fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()> {
        if self.end_pos + 1 > self.buffer.len() {
            return Err(());
        }
        self.buffer[self.end_pos] = data;
        self.end_pos += 1;
        Ok(())
    }

    pub fn read_byte_or_err(&mut self) -> Result<u8, ()> {
        if self.start_pos + 1 > self.end_pos {
            return Err(());
        }
        self.start_pos += 1;
        Ok(self.buffer[self.start_pos - 1])
    }

    /// Reset the TransportBuffer by copying the new slice into the buffer
    /// and resetting the start and end positions.
    pub fn reset_with_new_slice(mut self, new_slice: &[u8]) -> Result<Self, (Self, ())> {
        if new_slice.len() > self.buffer.len() {
            return Err((self, ()));
        }

        self.buffer[..new_slice.len()].copy_from_slice(new_slice);
        self.start_pos = 0;
        self.end_pos = new_slice.len();
        Ok(self)
    }
}

pub trait EncodedTransportPacket<'a>: Into<&'a mut [u8]> {}

pub trait RpcTxTransportPacket<'a> {
    type EncodedTransportPacket: EncodedTransportPacket<'a>;
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()>;
    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()>;
    fn new(buffer: &'a mut [u8]) -> Self;
    fn encode_packet(self) -> Result<Self::EncodedTransportPacket, ()>;
}

pub trait DecodedTransportPacket<'a>: Into<&'a mut [u8]> {}

pub trait RpcRxTransportPacket<'a>: Sized {
    type DecodedTransportPacket: DecodedTransportPacket<'a>;
    /// Attempt to form a new Rx transport buffer from the provided mutable buffer.
    /// Returns `Ok((buffer, Some(remaining_slice)))` if a portion of the buffer was
    /// successfully converted into a transport buffer.
    ///
    /// The remaining slice is the portion of the buffer that was not consumed by the transport buffer.
    ///
    /// Returns `Error((original_buffer, ()))` if the buffer could not be converted into a transport buffer.
    fn new(buffer: &'a mut [u8]) -> Result<(Self, Option<&'a mut [u8]>), (&'a mut [u8], ())>;
    fn decode_raw_packet(self) -> Result<Self::DecodedTransportPacket, ()>;
}
