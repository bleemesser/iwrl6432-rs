use core::cell::RefCell;
use core::convert::Infallible;
use core::future::poll_fn;
use core::task::Poll;

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use heapless::Deque;
use xwrl64xx_pac::{self as pac};

use crate::{clock, pinmux};

// SCI register offsets (identical for both instances).
const SCIGCR0: usize = 0x00;
const SCIGCR1: usize = 0x04;
const SCISETINT: usize = 0x0C;
const SCICLEARINT: usize = 0x10;
const SCICLEARINTLVL: usize = 0x18;
const SCIFLR: usize = 0x1C;
const SCICHAR: usize = 0x28;
const SCIBAUD: usize = 0x2C;
const SCIRD: usize = 0x34;
const SCITD: usize = 0x38;
const SCIPIO0: usize = 0x3C;

const GCR1_TIMING_ASYNC: u32 = 1 << 1;
const GCR1_CLOCK_INTERNAL: u32 = 1 << 5;
const GCR1_SW_NRESET: u32 = 1 << 7;
const GCR1_RXENA: u32 = 1 << 24;
const GCR1_TXENA: u32 = 1 << 25;
const PIO0_CLK_RX_TX: u32 = 0b111; // enable CLK + RX + TX pin functions
const INT_TX: u32 = 1 << 8;
const INT_RX: u32 = 1 << 9;
const FLR_TXRDY: u32 = 1 << 8;
const FLR_RXRDY: u32 = 1 << 9;
const CHAR_8BIT: u32 = 7; // SCICHAR = data bits - 1

/// Generate a full-duplex, interrupt-driven SCI UART instance in module `$mod`.
macro_rules! sci_uart {
    (
        $(#[$meta:meta])* $mod:ident,
        base = $base:expr, irq = $irq:ident, enable_clock = $enable_clock:path,
        tx = ($tx_pad:expr, $tx_mode:expr), rx = ($rx_pad:expr, $rx_mode:expr) $(,)?
    ) => {
        $(#[$meta])*
        pub mod $mod {
            use super::*;

            const BASE: usize = $base;

            static RX_FIFO: Mutex<RefCell<Deque<u8, 128>>> =
                Mutex::new(RefCell::new(Deque::new()));
            static RX_WAKER: AtomicWaker = AtomicWaker::new();
            static TX_FIFO: Mutex<RefCell<Deque<u8, 256>>> =
                Mutex::new(RefCell::new(Deque::new()));
            static TX_WAKER: AtomicWaker = AtomicWaker::new();

            #[inline]
            fn rd(off: usize) -> u32 {
                unsafe { core::ptr::read_volatile((BASE + off) as *const u32) }
            }
            #[inline]
            fn wr(off: usize, v: u32) {
                unsafe { core::ptr::write_volatile((BASE + off) as *mut u32, v) };
            }

            /// Zero-sized handle. State lives in hardware and module statics.
            #[derive(Clone, Copy)]
            pub struct Uart;

            impl Uart {
                /// Bring the UART up for 8N1 at `baud`, off
                /// [`clock::UART_CLK_HZ`].
                pub fn new(dp: &pac::Peripherals, baud: u32) -> Self {
                    $enable_clock(dp);
                    pinmux::configure(dp, $tx_pad, pinmux::pad_cfg($tx_mode));
                    pinmux::configure(dp, $rx_pad, pinmux::pad_cfg($rx_mode));

                    wr(SCIGCR0, 1); // module out of reset
                    wr(SCIGCR1, 0); // hold SW_NRESET low while configuring
                    wr(SCICLEARINT, 0xFFFF_FFFF);
                    wr(SCICLEARINTLVL, 0xFFFF_FFFF);
                    wr(SCIPIO0, PIO0_CLK_RX_TX);
                    wr(SCIGCR1, GCR1_TIMING_ASYNC | GCR1_CLOCK_INTERNAL | GCR1_RXENA | GCR1_TXENA);
                    wr(SCICHAR, CHAR_8BIT);
                    Self::set_baud(clock::UART_CLK_HZ, baud);
                    wr(SCIGCR1, rd(SCIGCR1) | GCR1_SW_NRESET);
                    Uart
                }

                /// Set `SCIBAUD` to `round(clk / (16 * baud))` minus one.
                pub fn set_baud(input_clk: u32, baud: u32) {
                    let step = baud * 16;
                    wr(SCIBAUD, (input_clk + step / 2) / step - 1);
                }

                /// Enable the RX interrupt and unmask the port's NVIC line.
                /// Requires global interrupts enabled, so flash boots must call
                /// [`crate::normalize_after_bootloader`] first.
                pub fn enable_interrupts() {
                    wr(SCISETINT, INT_RX);
                    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::$irq) };
                }

                /// Enqueue one byte for TX, spinning while the ring is full.
                pub fn write_byte(c: u8) {
                    while !push_tx(c) {
                        core::hint::spin_loop();
                    }
                }
            }

            /// Push one byte and enable the TX interrupt atomically. The TX
            /// interrupt is edge-triggered on TXRDY, so an idle transmitter
            /// produces no edge and the ISR is pended to prime the pump.
            #[inline]
            fn push_tx(c: u8) -> bool {
                critical_section::with(|cs| {
                    let ok = TX_FIFO.borrow(cs).borrow_mut().push_back(c).is_ok();
                    if ok {
                        wr(SCISETINT, INT_TX);
                        if rd(SCIFLR) & FLR_TXRDY != 0 {
                            cortex_m::peripheral::NVIC::pend(pac::Interrupt::$irq);
                        }
                    }
                    ok
                })
            }

            /// Await one received byte.
            pub async fn read_byte() -> u8 {
                poll_fn(|cx| {
                    RX_WAKER.register(cx.waker());
                    match critical_section::with(|cs| RX_FIFO.borrow(cs).borrow_mut().pop_front()) {
                        Some(b) => Poll::Ready(b),
                        None => Poll::Pending,
                    }
                })
                .await
            }

            /// Enqueue one byte for TX, yielding while the ring is full.
            pub async fn write_byte_async(c: u8) {
                poll_fn(|cx| {
                    TX_WAKER.register(cx.waker());
                    if push_tx(c) {
                        Poll::Ready(())
                    } else {
                        Poll::Pending
                    }
                })
                .await
            }

            impl core::fmt::Write for Uart {
                /// Write blocking and buffered, expanding `\n` to `\r\n`.
                fn write_str(&mut self, s: &str) -> core::fmt::Result {
                    for b in s.bytes() {
                        if b == b'\n' {
                            Uart::write_byte(b'\r');
                        }
                        Uart::write_byte(b);
                    }
                    Ok(())
                }
            }

            impl embedded_io::ErrorType for Uart {
                type Error = Infallible;
            }

            impl embedded_io_async::Read for Uart {
                async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
                    if buf.is_empty() {
                        return Ok(0);
                    }
                    buf[0] = read_byte().await;
                    Ok(1)
                }
            }

            impl embedded_io_async::Write for Uart {
                async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
                    for &b in buf {
                        write_byte_async(b).await;
                    }
                    Ok(buf.len())
                }

                async fn flush(&mut self) -> Result<(), Infallible> {
                    poll_fn(|cx| {
                        TX_WAKER.register(cx.waker());
                        let empty =
                            critical_section::with(|cs| TX_FIFO.borrow(cs).borrow().is_empty());
                        if empty { Poll::Ready(()) } else { Poll::Pending }
                    })
                    .await;
                    Ok(())
                }
            }

            /// Shared RX/TX handler (overrides the weak device.x alias).
            #[unsafe(no_mangle)]
            extern "C" fn $irq() {
                let flr = rd(SCIFLR);
                if flr & FLR_RXRDY != 0 {
                    let b = (rd(SCIRD) & 0xFF) as u8; // reading SCIRD clears RXRDY
                    critical_section::with(|cs| {
                        let _ = RX_FIFO.borrow(cs).borrow_mut().push_back(b);
                    });
                    RX_WAKER.wake();
                }
                if flr & FLR_TXRDY != 0 {
                    // pop and disable atomically, so a concurrent push is never
                    // stranded with TX disabled
                    critical_section::with(|cs| {
                        match TX_FIFO.borrow(cs).borrow_mut().pop_front() {
                            Some(b) => wr(SCITD, b as u32),
                            None => wr(SCICLEARINT, INT_TX),
                        }
                    });
                    TX_WAKER.wake();
                }
            }
        }
    };
}

sci_uart!(
    /// Console port on UARTB, the [`print!`] and [`println!`] target. This is
    /// the XDS110 backchannel on the BOOST. UARTA is unused.
    console,
    base = 0x57F7_F000,
    irq = APPSS_SCI2_INT0,
    enable_clock = clock::enable_uart1,
    tx = (pinmux::Pad::Ao, 2),
    rx = (pinmux::Pad::Ap, 2),
);

pub use console::Uart;
pub use console::{read_byte, write_byte_async};

/// `print!` to the console UART (buffered, blocking).
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut __w = $crate::uart::console::Uart;
        let _ = core::write!(__w, $($arg)*);
    }};
}

/// `println!` to the console UART (CRLF-terminated).
#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut __w = $crate::uart::console::Uart;
        let _ = core::writeln!(__w, $($arg)*);
    }};
}
