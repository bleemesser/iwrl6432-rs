// MCSPIA (APP_SPI_0, 0x53F7_F400) in peripheral (slave) mode: raw-data
// streaming to an external SPI master on the BOOST J2 header (board switch
// S1.6 ON routes pads AG/AH/AI/AJ there; the same mux steals GPIO5/USER_LED).
// Config: slave, ch0, SPI mode 0, CS active-low, 32-bit words, RX on D1 /
// TX on D0, TX FIFO enabled.
//
// The host-poll protocol is served entirely from the SPI ISR: batches are
// double-buffered (the stream task builds one while the other is armed or
// being clocked out), and the poll magic itself triggers channel reset +
// FIFO prime in the ISR. Response latency is therefore a few tens of us and
// independent of executor state - a task-level responder loses the race
// with the host's post-poll read deadline whenever a batch rebuild or a
// timer nap is in flight.

use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use crate::pac;

/// Poll magic the host master writes to request a batch.
pub const CMD_POLL: u32 = 0xC0FF_EE01;

/// Max words per batch (24.8 KB, 8 chirps of the largest 3032 B chunk).
/// Streaming is frame-paced: chirps arrive in per-frame clumps that the
/// capture ring absorbs, and the link drains 2 batches per frame; RAM spent
/// on ring slots (clump headroom) beats RAM spent on batch size.
pub const TX_BUF_WORDS: usize = 6208;

// Batch double buffer. The stream task writes a half only while it is not
// armed and not being served; the ISR reads a half only between arm() and
// push completion.
static mut TX_BUF: [[u32; TX_BUF_WORDS]; 2] = [[0; TX_BUF_WORDS]; 2];
/// Armed batch length in words per buffer; 0 = free for building.
static ARMED_LEN: [AtomicUsize; 2] = [AtomicUsize::new(0), AtomicUsize::new(0)];
const NO_BUF: usize = 2;
/// Buffer currently being pushed by the ISR (NO_BUF when idle or serving
/// the canned empty header).
static SERVING: AtomicUsize = AtomicUsize::new(NO_BUF);
static NEXT_SERVE: AtomicUsize = AtomicUsize::new(0); // ISR-owned
static NEXT_BUILD: AtomicUsize = AtomicUsize::new(0); // task-owned

// In-flight push state (ISR-owned between prime and completion).
static TX_SRC: AtomicUsize = AtomicUsize::new(0);
static TX_LEN: AtomicUsize = AtomicUsize::new(0);
static TX_POS: AtomicUsize = AtomicUsize::new(0);

/// Response to a poll with no armed batch: 5A A5 | total=0 | first_seq=0.
static EMPTY_HDR: [u32; 2] = [0x5AA5_0000, 0];

static SERVED: AtomicU32 = AtomicU32::new(0);
static EMPTY_POLLS: AtomicU32 = AtomicU32::new(0);
/// Recognized polls since boot (any poll whose magic landed word-aligned in
/// RX0). Freezes when the slave's word framing is desynced by a host that
/// abandoned a transaction mid-cycle - the idle watchdog samples this.
static POLLS: AtomicU32 = AtomicU32::new(0);
/// Polls that arrived while a previous batch was still being pushed: the
/// host gave up on it (read window passed); the batch is restarted.
static RETRIES: AtomicU32 = AtomicU32::new(0);
// Feed-ISR invocations that found the TX FIFO completely empty mid-push
// (words still pending, not at prime): true starvation, the master may have
// clocked garbage. The TX0_UNDERFLOW status latch is unusable for this - it
// also sets benignly at end-of-transfer (shift-register preload after the
// last word).
static TX_STARVED: AtomicU32 = AtomicU32::new(0);

const IRQ_TX_EMPTY: u32 = 1 << 0;
const IRQ_TX_UNDERFLOW: u32 = 1 << 1;
const IRQ_RX_FULL: u32 = 1 << 2;

/// Bring up MCSPIA as an SPI-mode-0 slave: clocks (inverted-gate convention,
/// 40 MHz OSC /1), pads, soft reset, channel-0 config, interrupt priorities.
/// Poll detection (RX interrupt) is live from here on.
pub fn init(dp: &pac::Peripherals) {
    dp.app_rcm
        .ipcfgclkgate1()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0000_01C0) });
    dp.app_rcm
        .app_spi_clkctl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !0x0FFF_FFFF) });

    // Pads, all function 0 = MCSPIA: AG=CLK, AI=MOSI, AJ=MISO, AH=CS0.
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

    // Ch0: mode 0, EPOL=1 (CS active low), WL=31 (32-bit), TRM=TX+RX,
    // DPE1=1 (TX on D0), IS=1 (RX on D1), FFEW=1 (TX FIFO).
    spi.ch0conf().write(|w| unsafe {
        w.bits((1 << 6) | (31 << 7) | (1 << 17) | (1 << 18) | (1 << 27))
    });
    // TX almost-empty trigger: TX0_EMPTY fires when the FIFO content drops
    // to <= AEL bytes. 31 = half-FIFO: 12.8 us of feed headroom at 20 MHz
    // (tx_starved() counts misses), and a low enough refill-IRQ rate
    // (~1/13 us during reads) that the chirp-copy ISR at prio 0x80 still
    // meets the 136 us intra-burst chirp spacing - at AEL=47 the doubled
    // IRQ overhead made big-chirp profiles lose chirp interrupts.
    spi.xferlevel().write(|w| unsafe { w.bits(31) });

    spi.ch0ctrl().write(|w| unsafe { w.bits(1) }); // enable channel

    // The SPI ISR must preempt everything: the chirp-copy ISR alone runs
    // ~25 us, longer than the 64-byte FIFO lasts at 15+ MHz, and the poll
    // response is primed in this ISR too. SPI stays at NVIC priority 0;
    // chirp-avail (34), console SCI (62) and RTI (43) drop to 0x80.
    const NVIC_IPR: usize = 0xE000_E400;
    for irq in [34u8, 62, 43] {
        unsafe { core::ptr::write_volatile((NVIC_IPR + irq as usize) as *mut u8, 0x80) };
    }

    spi.irqstatus().write(|w| unsafe { w.bits(0x7) });
    spi.irqenable().write(|w| unsafe { w.bits(IRQ_RX_FULL) });
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::APPSS_SPI_IRQ_REQ) };
}

/// The buffer the stream task should build the next batch into, if free.
/// `None` while both halves are armed/serving - retry after a yield.
pub fn build_slot() -> Option<&'static mut [u32; TX_BUF_WORDS]> {
    let nb = NEXT_BUILD.load(Ordering::Relaxed);
    if ARMED_LEN[nb].load(Ordering::Acquire) == 0 && SERVING.load(Ordering::Acquire) != nb {
        // Safe: the ISR only touches armed buffers, and this one is not.
        Some(unsafe { &mut (*(&raw mut TX_BUF))[nb] })
    } else {
        None
    }
}

/// Arm the batch built in the last [`build_slot`] (`len` words) for the ISR
/// to serve on the next poll, and advance the build cursor.
pub fn arm(len: usize) {
    let nb = NEXT_BUILD.load(Ordering::Relaxed);
    ARMED_LEN[nb].store(len, Ordering::Release);
    NEXT_BUILD.store(nb ^ 1, Ordering::Relaxed);
}

/// Drop armed-but-unserved batches (live reconfigure: their lengths/seqs are
/// stale) and realign the build cursor with the serve cursor.
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

/// `(pushed words, batch words)` of the in-flight push; equal when idle.
/// The stream task's stall watchdog samples this.
pub fn push_progress() -> (usize, usize) {
    (TX_POS.load(Ordering::Relaxed), TX_LEN.load(Ordering::Relaxed))
}

/// `true` while the ISR is pushing a real (non-empty-header) batch.
pub fn is_serving() -> bool {
    SERVING.load(Ordering::Acquire) != NO_BUF
}

/// `true` if any batch is armed (including one currently being served).
pub fn any_armed() -> bool {
    ARMED_LEN.iter().any(|a| a.load(Ordering::Acquire) != 0)
}

/// Flush the channel and go back to poll-listening; `reset_channel` re-aligns
/// the shift register to the next CS frame. The armed batch is kept and
/// re-served on the next poll. Recovers both wedges the stream task watches
/// for: a host that stopped clocking mid-batch and never re-polled (e.g. the
/// capture tool was killed), and a slave whose word framing desynced after a
/// host abandoned a transaction mid-cycle, so the poll magic no longer lands
/// word-aligned and the ISR never recognizes another poll. Harmless when the
/// port is merely idle (empty FIFOs), but it discards an in-flight push, so
/// only call once [`push_progress`] has been stalled for a long time or
/// [`is_serving`] is false.
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

/// Recognized polls since boot (see `POLLS`). The stream task's idle watchdog
/// samples this to detect a framing-desynced slave that will never see another
/// poll and re-inits the channel.
pub fn polls() -> u32 {
    POLLS.load(Ordering::Relaxed)
}

/// Completed (fully pushed) non-empty batches since boot.
pub fn served() -> u32 {
    SERVED.load(Ordering::Relaxed)
}

/// Polls answered with the canned empty header (no batch armed).
pub fn empty_polls() -> u32 {
    EMPTY_POLLS.load(Ordering::Relaxed)
}

/// Batches restarted because the host re-polled mid-push (see `RETRIES`).
pub fn retries() -> u32 {
    RETRIES.load(Ordering::Relaxed)
}

/// Mid-push TX FIFO starvation events since boot (see `TX_STARVED`).
pub fn tx_starved() -> u32 {
    TX_STARVED.load(Ordering::Relaxed)
}

/// Disable + re-enable channel 0, flushing both FIFOs and the shift register.
pub fn reset_channel() {
    let spi = unsafe { &*pac::AppSpi0::PTR };
    spi.ch0ctrl().write(|w| unsafe { w.bits(0) });
    cortex_m::asm::delay(64);
    spi.ch0ctrl().write(|w| unsafe { w.bits(1) });
}

/// Push from the in-flight source into the TX FIFO until full or done. At
/// push completion the consumed buffer is released and the ISR goes back to
/// poll-listening (the FIFO tail drains without further help).
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
        // Ack TX and any RX-full latched while pushing, then listen for the
        // next poll (RX events; the last <=64 B drain by themselves).
        spi.irqstatus().write(|w| unsafe { w.bits(IRQ_TX_EMPTY | IRQ_RX_FULL) });
        spi.irqenable().write(|w| unsafe { w.bits(IRQ_RX_FULL) });
    } else {
        spi.irqstatus().write(|w| unsafe { w.bits(IRQ_TX_EMPTY) });
        spi.irqenable().write(|w| unsafe { w.bits(IRQ_TX_EMPTY) });
    }
}

/// SPI ISR: poll detection + response prime + TX FIFO refill.
#[unsafe(no_mangle)]
extern "C" fn APPSS_SPI_IRQ_REQ() {
    let spi = unsafe { &*pac::AppSpi0::PTR };

    // Drain received words; only the poll magic means anything (the host's
    // read clocks shift junk into RX0, and reading keeps RXS from sticking).
    let mut poll = false;
    while spi.ch0stat().read().rxs().bit_is_set() {
        if spi.rx0().read().bits() == CMD_POLL {
            poll = true;
        }
    }

    if poll {
        POLLS.fetch_add(1, Ordering::Relaxed);
        // Answer right here. A poll while a push is outstanding means the
        // host abandoned that read; restart the same batch (still armed).
        if SERVING.load(Ordering::Relaxed) != NO_BUF {
            RETRIES.fetch_add(1, Ordering::Relaxed);
        }
        // Let the poll write's CS frame end before resetting the channel.
        // Keep this SHORT: it blocks at NVIC prio 0, and chirp IRQs whose
        // entry is delayed past the intra-burst chirp spacing pend-merge and
        // flip the ping/pong parity (full-Doppler "blast" corruption). The
        // batched-MPSSE host deasserts CS sub-us after the poll's last byte.
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

    // TX refill. FIFO completely empty mid-push = the feed lost the race.
    let (pos, len) = (TX_POS.load(Ordering::Relaxed), TX_LEN.load(Ordering::Relaxed));
    if pos > 0 && pos < len && spi.ch0stat().read().txffe().bit_is_set() {
        TX_STARVED.fetch_add(1, Ordering::Relaxed);
    }
    push(spi);
}
