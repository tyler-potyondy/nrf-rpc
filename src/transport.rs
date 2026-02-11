//! Transport layer traits for NRF RPC
//!
//! This module defines the transport abstraction that users must implement
//! to provide byte-level communication for the RPC protocol.

// async_fn_in_trait is expected for embedded no_std usage
#![allow(async_fn_in_trait)]

use core::fmt;

/// Error trait for transport implementations
pub trait TransportError: fmt::Debug + fmt::Display {}

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
    
    /// Write bytes to the transport
    ///
    /// Should block until all bytes are written or an error occurs.
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    
    /// Read bytes from the transport into the provided buffer
    ///
    /// Returns the number of bytes read. May return fewer bytes than
    /// the buffer size if data is not immediately available.
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error>;
}
