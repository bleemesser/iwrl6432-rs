use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::pac;

/// Poll magic the host master writes to request a batch.
pub const CMD_POLL: u32 = 0xC0FF_EE01;

/// Max words per batch, sized for 8 chirps of the largest 3032 B chunk. The
/// link drains 2 batches per frame, so RAM is better spent on ring slots than
/// on a bigger batch.
pub const TX_BUF_WORDS: usize = 6208;

// the task writes a half only while it is neither armed nor being served, and
// the ISR reads a half only between arm() and push completion
static mut TX_BUF: [[u32; TX_BUF_WORDS]; 2] = [[0; TX_BUF_WORDS]; 2];
/// Armed batch length in words per buffer, 0 when free for building.
static ARMED_LEN: [AtomicUsize; 2] = [AtomicUsize::new(0), AtomicUsize::new(0)];
const NO_BUF: usize = 2;
/// Buffer the ISR is pushing, `NO_BUF` when idle or serving the empty header.
static SERVING: AtomicUsize = AtomicUsize::new(NO_BUF);
static NEXT_SERVE: AtomicUsize = AtomicUsize::new(0); // ISR-owned
static NEXT_BUILD: AtomicUsize = AtomicUsize::new(0); // task-owned

// In-flight push state (ISR-owned between prime and completion).
static TX_SRC: AtomicUsize = AtomicUsize::new(0);
static TX_LEN: AtomicUsize = AtomicUsize::new(0);
static TX_POS: AtomicUsize = AtomicUsize::new(0);

/// Response to a poll with no armed batch.
static EMPTY_HDR: [u32; 2] = [0x5AA5_0000, 0];

static SERVED: AtomicU32 = AtomicU32::new(0);
static EMPTY_POLLS: AtomicU32 = AtomicU32::new(0);
/// Polls whose magic landed word-aligned in RX0. Freezes once word framing
/// desyncs, which is how the idle watchdog spots a wedge.
static POLLS: AtomicU32 = AtomicU32::new(0);
/// Polls that arrived mid-push, meaning the host gave up and the batch restarts.
static RETRIES: AtomicU32 = AtomicU32::new(0);
// TX0_UNDERFLOW is unusable for this, since it also sets benignly at
// end-of-transfer when the shift register preloads after the last word
static TX_STARVED: AtomicU32 = AtomicU32::new(0);

const IRQ_TX_EMPTY: u32 = 1 << 0;
const IRQ_TX_UNDERFLOW: u32 = 1 << 1;
const IRQ_RX_FULL: u32 = 1 << 2;

/// Bring up MCSPIA as an SPI-mode-0 slave on channel 0, CS active-low, 32-bit
/// words. Poll detection is live from here on. Reaches the J2 header only with
/// board switch S1.6 on, which also steals GPIO5/USER_LED.
pub fn init(dp: &pac::Peripherals) {
    dp.app_rcm
        .ipcfgclkgate1()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0000_01C0) });
    dp.app_rcm
        .app_spi_clkctl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0FFF_FFFF) });

    // function 0 is MCSPIA: AG is CLK, AI MOSI, AJ MISO, AH CS0
    use crate::pinmux::{configure, pad_cfg, Pad};
    configure(dp, Pad::Ag, pad_cfg(0));
    configure(dp, Pad::Ai, pad_cfg(0));
    configure(dp, Pad::Aj, pad_cfg(0));
    configure(dp, Pad::Ah, pad_cfg(0));

    let spi = &dp.app_spi_0;

    spi.sysconfig().write(|w| unsafe { w.bits(1 << 1) }); // soft reset
    while spi.sysstatus().read().bits() & 0x1 == 0 {}
    spi.sysconfig().write(|w| unsafe { w.bits(1 << 3) }); // no-idle

    spi.modulctrl().write(|w| unsafe { w.bits(1 << 2) }); // slave (MS=1)

    // EPOL, WL=31, TRM=TX+RX, DPE1 (TX on D0), IS (RX on D1), FFEW
    spi.ch0conf().write(|w| unsafe {
        w.bits((1 << 6) | (31 << 7) | (1 << 17) | (1 << 18) | (1 << 27))
    });
    // half-FIFO almost-empty trigger, giving 12.8 us of feed headroom at 20 MHz
    // without the refill-IRQ rate starving the chirp-copy ISR (AEL=47 loses chirps)
    spi.xferlevel().write(|w| unsafe { w.bits(31) });

    spi.ch0ctrl().write(|w| unsafe { w.bits(1) }); // enable channel

    // SPI stays at priority 0 and must preempt everything, since the chirp-copy
    // ISR alone runs ~25 us, longer than the 64-byte FIFO lasts at 15+ MHz
    const NVIC_IPR: usize = 0xE000_E400;
    for irq in [34u8, 62, 43] {
        unsafe { core::ptr::write_volatile((NVIC_IPR + irq as usize) as *mut u8, 0x80) };
    }

    spi.irqstatus().write(|w| unsafe { w.bits(0x7) });
    spi.irqenable().write(|w| unsafe { w.bits(IRQ_RX_FULL) });
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::APPSS_SPI_IRQ_REQ) };
}

/// Get the buffer to build the next batch into, or `None` while both halves are
/// armed or serving, in which case retry after a yield.
pub fn build_slot() -> Option<&'static mut [u32; TX_BUF_WORDS]> {
    let nb = NEXT_BUILD.load(Ordering::Relaxed);
    if ARMED_LEN[nb].load(Ordering::Acquire) == 0 && SERVING.load(Ordering::Acquire) != nb {
        // sound because the ISR only touches armed buffers, and this one is not
        Some(unsafe { &mut (*(&raw mut TX_BUF))[nb] })
    } else {
        None
    }
}

/// Arm the `len`-word batch built in the last [`build_slot`] for the ISR to
/// serve on the next poll, and advance the build cursor.
pub fn arm(len: usize) {
    let nb = NEXT_BUILD.load(Ordering::Relaxed);
    ARMED_LEN[nb].store(len, Ordering::Release);
    NEXT_BUILD.store(nb ^ 1, Ordering::Relaxed);
}

/// Drop armed-but-unserved batches and realign the build cursor with the serve
/// cursor. Used on a live reconfigure, where their lengths and seqs go stale.
pub fn flush_armed() {
    let s = SERVING.load(Ordering::Acquire);
    for (i, a) in ARMED_LEN.iter().enumerate() {
        if s != i {
            a.store(0, Ordering::Release);
        }
    }
    NEXT_BUILD.store(
        if s != NO_BUF { s ^ 1 } else { NEXT_SERVE.load(Ordering::Relaxed) },
        Ordering::Relaxed,
    );
}

/// Report `(pushed words, batch words)` of the in-flight push, equal when idle.
pub fn push_progress() -> (usize, usize) {
    (TX_POS.load(Ordering::Relaxed), TX_LEN.load(Ordering::Relaxed))
}

/// Report whether the ISR is pushing a real batch rather than the empty header.
pub fn is_serving() -> bool {
    SERVING.load(Ordering::Acquire) != NO_BUF
}

/// Report whether any batch is armed, including one being served.
pub fn any_armed() -> bool {
    ARMED_LEN.iter().any(|a| a.load(Ordering::Acquire) != 0)
}

/// Flush the channel and go back to poll-listening, re-aligning the shift
/// register to the next CS frame. The armed batch is kept and re-served on the
/// next poll. This discards an in-flight push, so only call it once
/// [`push_progress`] has been stalled a long time or [`is_serving`] is false.
pub fn force_idle() {
    let spi = unsafe { &*pac::AppSpi0::PTR };
    spi.irqenable().write(|w| unsafe { w.bits(0) });
    SERVING.store(NO_BUF, Ordering::Release);
    TX_POS.store(0, Ordering::Relaxed);
    TX_LEN.store(0, Ordering::Relaxed);
    reset_channel();
    spi.irqstatus().write(|w| unsafe { w.bits(0x7) });
    spi.irqenable().write(|w| unsafe { w.bits(IRQ_RX_FULL) });
}

/// Count recognized polls since boot.
pub fn polls() -> u32 {
    POLLS.load(Ordering::Relaxed)
}

/// Count fully pushed non-empty batches since boot.
pub fn served() -> u32 {
    SERVED.load(Ordering::Relaxed)
}

/// Count polls answered with the empty header because no batch was armed.
pub fn empty_polls() -> u32 {
    EMPTY_POLLS.load(Ordering::Relaxed)
}

/// Count batches restarted because the host re-polled mid-push.
pub fn retries() -> u32 {
    RETRIES.load(Ordering::Relaxed)
}

/// Count mid-push TX FIFO starvations since boot, where the master may have
/// clocked garbage.
pub fn tx_starved() -> u32 {
    TX_STARVED.load(Ordering::Relaxed)
}

/// Cycle channel 0, flushing both FIFOs and the shift register.
pub fn reset_channel() {
    let spi = unsafe { &*pac::AppSpi0::PTR };
    spi.ch0ctrl().write(|w| unsafe { w.bits(0) });
    cortex_m::asm::delay(64);
    spi.ch0ctrl().write(|w| unsafe { w.bits(1) });
}

/// Push from the in-flight source into the TX FIFO until full or done. On
/// completion the consumed buffer is released and the ISR returns to
/// poll-listening, since the FIFO tail drains without further help.
fn push(spi: &pac::app_spi_0::RegisterBlock) {
    let src = TX_SRC.load(Ordering::Relaxed) as *const u32;
    let len = TX_LEN.load(Ordering::Relaxed);
    let mut pos = TX_POS.load(Ordering::Relaxed);
    while pos < len && spi.ch0stat().read().txfff().bit_is_clear() {
        let w = unsafe { core::ptr::read(src.add(pos)) };
        spi.tx0().write(|r| unsafe { r.bits(w) });
        pos += 1;
    }
    TX_POS.store(pos, Ordering::Relaxed);
    if pos >= len {
        let cur = SERVING.swap(NO_BUF, Ordering::AcqRel);
        if cur != NO_BUF {
            ARMED_LEN[cur].store(0, Ordering::Release);
            NEXT_SERVE.store(cur ^ 1, Ordering::Relaxed);
            SERVED.fetch_add(1, Ordering::Relaxed);
        }
        // ack TX and any RX-full latched while pushing, then listen for the next poll
        spi.irqstatus().write(|w| unsafe { w.bits(IRQ_TX_EMPTY | IRQ_RX_FULL) });
        spi.irqenable().write(|w| unsafe { w.bits(IRQ_RX_FULL) });
    } else {
        spi.irqstatus().write(|w| unsafe { w.bits(IRQ_TX_EMPTY) });
        spi.irqenable().write(|w| unsafe { w.bits(IRQ_TX_EMPTY) });
    }
}

/// Detect polls, prime the response and refill the TX FIFO. Serving the whole
/// protocol from the ISR keeps response latency independent of executor state.
#[unsafe(no_mangle)]
extern "C" fn APPSS_SPI_IRQ_REQ() {
    let spi = unsafe { &*pac::AppSpi0::PTR };

    // the host's read clocks shift junk into RX0, so only the magic means
    // anything, and draining keeps RXS from sticking
    let mut poll = false;
    while spi.ch0stat().read().rxs().bit_is_set() {
        if spi.rx0().read().bits() == CMD_POLL {
            poll = true;
        }
    }

    if poll {
        POLLS.fetch_add(1, Ordering::Relaxed);
        // a poll mid-push means the host abandoned that read, so restart the batch
        if SERVING.load(Ordering::Relaxed) != NO_BUF {
            RETRIES.fetch_add(1, Ordering::Relaxed);
        }
        // let the poll write's CS frame end before resetting the channel. keep this
        // short, since it blocks at priority 0 and chirp IRQs delayed past the
        // intra-burst spacing pend-merge and flip the ping/pong parity
        cortex_m::asm::delay(crate::clock::CORE_CLK_HZ / 200_000); // ~5 us
        reset_channel();
        let nb = NEXT_SERVE.load(Ordering::Relaxed);
        let len = ARMED_LEN[nb].load(Ordering::Acquire);
        let (src, len) = if len != 0 {
            SERVING.store(nb, Ordering::Release);
            (unsafe { (*(&raw const TX_BUF))[nb].as_ptr() }, len)
        } else {
            EMPTY_POLLS.fetch_add(1, Ordering::Relaxed);
            SERVING.store(NO_BUF, Ordering::Release);
            (EMPTY_HDR.as_ptr(), EMPTY_HDR.len())
        };
        TX_SRC.store(src as usize, Ordering::Relaxed);
        TX_LEN.store(len, Ordering::Relaxed);
        TX_POS.store(0, Ordering::Relaxed);
        spi.irqstatus()
            .write(|w| unsafe { w.bits(IRQ_TX_EMPTY | IRQ_TX_UNDERFLOW | IRQ_RX_FULL) });
        push(spi);
        return;
    }

    // an empty FIFO mid-push means the feed lost the race
    let (pos, len) = (TX_POS.load(Ordering::Relaxed), TX_LEN.load(Ordering::Relaxed));
    if pos > 0 && pos < len && spi.ch0stat().read().txffe().bit_is_set() {
        TX_STARVED.fetch_add(1, Ordering::Relaxed);
    }
    push(spi);
}
