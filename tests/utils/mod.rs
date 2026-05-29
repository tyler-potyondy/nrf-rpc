extern crate alloc;

use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll, RawWaker, RawWakerVTable};
use nrf_rpc::{TransportError, uart_transport::Uart};

/// Simple blocking executor for tests - only works for futures that are immediately ready
pub fn block_on<F: Future + Unpin>(mut fut: F) -> F::Output {
    unsafe fn noop_clone(_: *const ()) -> RawWaker {
        noop_raw_waker()
    }

    unsafe fn noop(_: *const ()) {}

    fn noop_raw_waker() -> RawWaker {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(noop_clone, noop, noop, noop);
        RawWaker::new(ptr::null(), &VTABLE)
    }

    let waker = unsafe { core::task::Waker::from_raw(noop_raw_waker()) };
    let mut ctx = Context::from_waker(&waker);

    loop {
        match Pin::new(&mut fut).poll(&mut ctx) {
            Poll::Ready(val) => return val,
            Poll::Pending => panic!("Future should be immediately ready in tests"),
        }
    }
}

/// Mock UART byte source for unit tests — records transmitted bytes and returns
/// no data on reads. Wrap with [`nrf_rpc::uart_transport::UartTransport`] to
/// obtain a full [`nrf_rpc::AsyncTransport`].
#[derive(Clone)]
pub struct MockUart {
    pub transmitted: Rc<RefCell<Vec<u8>>>,
}

#[derive(Debug)]
pub struct MockError;

impl TransportError for MockError {}

impl Uart for MockUart {
    type Error = MockError;

    async fn read(&mut self, _buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }

    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        self.transmitted.borrow_mut().extend_from_slice(data);
        Ok(data.len())
    }

    async fn delay_ms(&mut self, _ms: u32) {}

    fn has_buffered_data(&mut self) -> bool {
        false
    }
}
