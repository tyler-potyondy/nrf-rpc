//! Transport layer traits for NRF RPC
//!
//! This module defines the transport abstraction that users must implement
//! to provide byte-level communication for the RPC protocol.

// async_fn_in_trait is expected for embedded no_std usage
#![allow(async_fn_in_trait)]

use core::fmt;

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
    type TransportBuffer<'a, const N: usize>: RpcTransportBuffer<'a, N>;

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

pub struct TransportBuffer<'a, const N: usize> {
    buffer: &'a mut [u8; N],
    start_pos: usize,
    end_pos: usize,
}

/// From a mutable buffer to a Transport Buffer
impl<'a, const N: usize> From<&'a mut [u8; N]> for TransportBuffer<'a, N> {
    fn from(value: &'a mut [u8; N]) -> Self {
        Self {
            buffer: value,
            start_pos: 0,
            end_pos: N,
        }
    }
}

impl<'a, const N: usize> From<&'a TransportBuffer<'a, N>> for &'a [u8] {
    fn from(value: &'a TransportBuffer<'a, N>) -> Self {
        if value.start_pos >= value.end_pos {
            panic!(
                "Start position is greater than end position {:02x?}, start_pos: {}, end_pos: {}",
                value.buffer, value.start_pos, value.end_pos
            );
        }
        &value.buffer[value.start_pos..value.end_pos]
    }
}
impl<'a, const N: usize> From<TransportBuffer<'a, N>> for &'a mut [u8] {
    fn from(value: TransportBuffer<'a, N>) -> Self {
        if value.start_pos >= value.end_pos {
            panic!(
                "Start position is greater than end position {:02x?}, start_pos: {}, end_pos: {}",
                value.buffer, value.start_pos, value.end_pos
            );
        }
        &mut value.buffer[value.start_pos..value.end_pos]
    }
}

impl<'a, const N: usize> TransportBuffer<'a, N> {
    /// Creates a new RpcTransportBuffer from a mutable buffer.
    ///
    /// The buffer is assumed to be uninitialized.
    pub fn new(buffer: &'a mut [u8; N]) -> Self {
        Self {
            buffer,
            start_pos: 0,
            end_pos: 0,
        }
    }
}

impl<'a, const N: usize> TransportBuffer<'a, N> {
    pub fn remaining_len(&self) -> usize {
        N - self.end_pos
    }

    pub fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()> {
        if self.end_pos + data.len() > N {
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
}

pub trait RpcTransportBuffer<'a, const N: usize>: TryInto<&'a mut [u8]> {
    fn write_slice_into_or_err(&mut self, data: &[u8]) -> Result<(), ()>;
    fn write_byte_into_or_err(&mut self, data: u8) -> Result<(), ()>;
    fn new(buffer: &'a mut [u8; N]) -> Self;
}
