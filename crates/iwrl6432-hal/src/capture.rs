// Continuous raw-ADC capture: the DFE writes each chirp into the ADCBUF's
// ping/pong halves (alternating per chirp) and pulses NVIC IRQ 34. The ISR
// here copies the finished half into a static ring; a consumer pops whole
// chirps. Ping/pong is selected by chirp parity (chirp 0 of a burst = ping),
// so `chirps_per_burst` must be EVEN for parity to hold across bursts.

use core::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::sensor;

/// On-device sample encoding applied in the drain ISR.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// 2 samples -> 3 bytes (12-bit two's complement), exact for in-range
    /// samples. Out-of-range samples (never seen from the 12-bit ADC) alias;
    /// counted in [`out_of_range`]. The baseline streaming format.
    Packed12,
    /// Random subsampling for host-side compressed-sensing reconstruction:
    /// of every 16 sample pairs, keep `k` (1..=15), Packed12-encoded. The
    /// kept-pair mask is derived from `seq` alone ([`mask_fill`]), so the
    /// host regenerates it with no index side-channel; wire rate scales by
    /// k/16 and the reconstructed profile loses 10*log10(16/k) dB SNR.
    Sub12(u8),
}

/// Mask granularity for [`Format::Sub12`]: pairs are drawn per block of
/// this many (stratified, so kept samples never cluster badly).
pub const SUB12_BLOCK_PAIRS: usize = 16;

/// Ring capacity in chirps. 20 x 3072 B = 61 KB of APPSS RAM (RAM3 is
/// 128 KB total, shared with the two SPI batch buffers and the stack).
/// Must absorb a whole frame's chirp clump: profiles emit chirps in bursts
/// (e.g. 16 chirps in 8 ms of a 40 ms frame), and the SPI link drains at
/// most one batch per host poll cycle in the gaps.
const RING_CHIRPS: usize = 20;
/// Per-chirp storage; `init` asserts the configured geometry fits.
/// 3072 allows 680 samples x 3 RX packed 12-bit, keep 7/16 at 1536 samples
/// (3024 B) or keep 5/16 at 2048 (2880 B). Consumers size their pop buffers
/// with this.
pub const MAX_CHIRP_BYTES: usize = 3072;

// Lock-free SPSC ring: the chirp ISR is the only producer, the streaming
// task the only consumer. No critical section is held during the (up to
// ~70 us) copy/pack, so the higher-priority SPI feed ISR can preempt it -
// holding PRIMASK that long underruns the 20 MHz TX FIFO.
struct RingCell(core::cell::UnsafeCell<Ring>);
unsafe impl Sync for RingCell {}
static RING: RingCell = RingCell(core::cell::UnsafeCell::new(Ring::new()));
static HEAD: AtomicUsize = AtomicUsize::new(0); // ISR-owned
static TAIL: AtomicUsize = AtomicUsize::new(0); // consumer-owned
static COUNT: AtomicUsize = AtomicUsize::new(0);
static WAKER: AtomicWaker = AtomicWaker::new();
static DROPPED: AtomicU32 = AtomicU32::new(0);
static SEEN: AtomicU32 = AtomicU32::new(0);
static OOR: AtomicU32 = AtomicU32::new(0);
static GENERATION: AtomicU32 = AtomicU32::new(0);

// Capture geometry, set by `init` before the IRQ is unmasked (read in the ISR).
// FORMAT holds the kept pairs per block: SUB12_BLOCK_PAIRS = full Packed12,
// less = Sub12(k).
static NUM_SAMPLES: AtomicUsize = AtomicUsize::new(0);
static RX_COUNT: AtomicUsize = AtomicUsize::new(0);
static FORMAT: AtomicU8 = AtomicU8::new(SUB12_BLOCK_PAIRS as u8);

// Kept-pair indices for the chirp being drained (ISR-only, single producer).
// Sized for the most pairs a slot can hold: MAX_CHIRP_BYTES / 3 B per pair.
struct MaskCell(core::cell::UnsafeCell<[u16; MAX_CHIRP_BYTES / 3]>);
unsafe impl Sync for MaskCell {}
static MASK: MaskCell = MaskCell(core::cell::UnsafeCell::new([0; MAX_CHIRP_BYTES / 3]));

struct Slot {
    seq: u32,
    len: usize,
    data: [u8; MAX_CHIRP_BYTES],
}

struct Ring {
    slots: [Slot; RING_CHIRPS],
}

impl Ring {
    const fn new() -> Self {
        const EMPTY: Slot = Slot { seq: 0, len: 0, data: [0; MAX_CHIRP_BYTES] };
        Ring { slots: [EMPTY; RING_CHIRPS] }
    }
}

/// One drained chirp, as handed to the consumer.
pub struct Chirp {
    /// Monotonic chirp index since [`init`]; drops show as gaps host-side.
    pub seq: u32,
    /// Payload length in bytes (see [`payload_bytes`]).
    pub len: usize,
}

/// Record the capture geometry and unmask the chirp-avail interrupt. Call
/// after [`sensor::adcbuf_config`], before [`sensor::start`]. `rx_count` is
/// the number of enabled RX regions (drained contiguously in RX order).
pub fn init(num_adc_samples: u16, rx_count: usize, format: Format) {
    let bytes = payload_bytes(num_adc_samples as usize, rx_count, format);
    assert!(bytes <= MAX_CHIRP_BYTES, "chirp larger than capture slot");
    assert!(bytes % 4 == 0, "chirp payload must be whole SPI words");
    let keep = match format {
        Format::Packed12 => {
            assert!(num_adc_samples % 2 == 0, "Packed12 packs sample pairs");
            SUB12_BLOCK_PAIRS as u8
        }
        Format::Sub12(k) => {
            assert!(
                (1..SUB12_BLOCK_PAIRS as u8).contains(&k),
                "Sub12 keeps 1..=15 of 16 pairs"
            );
            assert!(
                num_adc_samples as usize % (2 * SUB12_BLOCK_PAIRS) == 0,
                "Sub12 needs whole 16-pair blocks"
            );
            k
        }
    };
    NUM_SAMPLES.store(num_adc_samples as usize, Ordering::Relaxed);
    RX_COUNT.store(rx_count, Ordering::Relaxed);
    FORMAT.store(keep, Ordering::Relaxed);
    SEEN.store(0, Ordering::Relaxed);
    DROPPED.store(0, Ordering::Relaxed);
    OOR.store(0, Ordering::Relaxed);
    GENERATION.fetch_add(1, Ordering::Relaxed);
    // Quiesce the producer while the indexes reset (live reconfigure).
    cortex_m::peripheral::NVIC::mask(
        crate::pac::Interrupt::MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN,
    );
    HEAD.store(0, Ordering::Relaxed);
    TAIL.store(0, Ordering::Relaxed);
    COUNT.store(0, Ordering::Release);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(
            crate::pac::Interrupt::MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN,
        )
    };
}

/// `(chirps seen, chirps dropped)` since [`init`].
pub fn stats() -> (u32, u32) {
    (SEEN.load(Ordering::Relaxed), DROPPED.load(Ordering::Relaxed))
}

/// Samples that did not fit 12 bits during [`Format::Packed12`] packing.
/// Nonzero means the stream is corrupt (should never happen: the ADC is
/// 12-bit).
pub fn out_of_range() -> u32 {
    OOR.load(Ordering::Relaxed)
}

/// Bumped by every [`init`]; consumers use it to discard batches built for
/// a previous configuration.
pub fn generation() -> u32 {
    GENERATION.load(Ordering::Relaxed)
}

/// Enabled RX regions per chirp (set by [`init`]).
pub fn rx_count() -> usize {
    RX_COUNT.load(Ordering::Relaxed)
}

/// ADC samples per chirp per RX (set by [`init`]).
pub fn num_samples() -> usize {
    NUM_SAMPLES.load(Ordering::Relaxed)
}

/// The encoding chirps are stored/streamed in.
pub fn format() -> Format {
    match FORMAT.load(Ordering::Relaxed) {
        k if (k as usize) < SUB12_BLOCK_PAIRS => Format::Sub12(k),
        _ => Format::Packed12,
    }
}

/// Payload bytes per chirp for a geometry/format (3 B per kept sample pair).
pub fn payload_bytes(samples: usize, rx_count: usize, format: Format) -> usize {
    match format {
        Format::Packed12 => rx_count * samples * 3 / 2,
        Format::Sub12(k) => rx_count * (samples / 2 / SUB12_BLOCK_PAIRS) * k as usize * 3,
    }
}

/// Fill `out` with the kept-pair indices for chirp `seq`: xorshift32 seeded
/// from `seq` alone; per 16-pair block a partial Fisher-Yates draws `keep`
/// pairs, emitted ascending. The host mirrors this exactly
/// (tools/spi_capture.py `mask_indices`) - change both or neither.
fn mask_fill(seq: u32, pairs: usize, keep: usize, out: &mut [u16]) -> usize {
    let mut s = (seq ^ 0xA5A5_5A5A).wrapping_mul(0x9E37_79B9) | 1;
    let mut rng = || {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        s
    };
    let mut n = 0;
    for block in 0..pairs / SUB12_BLOCK_PAIRS {
        let mut idx: [u16; SUB12_BLOCK_PAIRS] = core::array::from_fn(|i| i as u16);
        for j in 0..keep {
            let r = j + (rng() as usize % (SUB12_BLOCK_PAIRS - j));
            idx.swap(j, r);
        }
        idx[..keep].sort_unstable();
        for &i in &idx[..keep] {
            out[n] = (block * SUB12_BLOCK_PAIRS) as u16 + i;
            n += 1;
        }
    }
    n
}

/// Await the next chirp and copy it into `buf` (at least the chirp payload size).
pub async fn read_chirp(buf: &mut [u8]) -> Chirp {
    core::future::poll_fn(|cx| match pop_chirp(buf) {
        Some(c) => core::task::Poll::Ready(c),
        None => {
            WAKER.register(cx.waker());
            core::task::Poll::Pending
        }
    })
    .await
}

/// Non-blocking pop of the oldest chirp into `buf`; `None` if the ring is empty.
pub fn pop_chirp(buf: &mut [u8]) -> Option<Chirp> {
    if COUNT.load(Ordering::Acquire) == 0 {
        return None;
    }
    let tail = TAIL.load(Ordering::Relaxed);
    // Safe: single consumer; the producer never touches slots[tail] while
    // COUNT > 0 says it is ours.
    let slot = unsafe { &(*RING.0.get()).slots[tail] };
    let (seq, len) = (slot.seq, slot.len);
    buf[..len].copy_from_slice(&slot.data[..len]);
    TAIL.store((tail + 1) % RING_CHIRPS, Ordering::Relaxed);
    COUNT.fetch_sub(1, Ordering::Release);
    Some(Chirp { seq, len })
}

/// Chirps currently buffered.
pub fn pending() -> usize {
    COUNT.load(Ordering::Acquire)
}

/// `true` when the ring is close enough to full that the consumer should
/// drain it now to avoid drops.
pub fn nearly_full() -> bool {
    COUNT.load(Ordering::Acquire) >= RING_CHIRPS - 4
}

/// Payload bytes per chirp for the configured geometry and format.
pub fn chirp_len() -> usize {
    payload_bytes(
        NUM_SAMPLES.load(Ordering::Relaxed),
        RX_COUNT.load(Ordering::Relaxed),
        format(),
    )
}

/// Sequence number of the oldest buffered chirp (0 if empty).
pub fn next_seq() -> u32 {
    if COUNT.load(Ordering::Acquire) == 0 {
        return 0;
    }
    unsafe { (*RING.0.get()).slots[TAIL.load(Ordering::Relaxed)].seq }
}

/// Chirp-available ISR (NVIC IRQ 34): drain the just-finished ping/pong half
/// into the ring, packing per the configured [`Format`] (~35 us packed for a
/// 256-sample x 3-RX chirp; Sub12 is cheaper still, it reads only the kept
/// words). If the ring is full the chirp is dropped and counted.
#[unsafe(no_mangle)]
extern "C" fn MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN() {
    let seq = SEEN.fetch_add(1, Ordering::Relaxed);
    let samples = NUM_SAMPLES.load(Ordering::Relaxed);
    let rx_count = RX_COUNT.load(Ordering::Relaxed);
    if samples == 0 {
        return;
    }
    sensor::adcbuf_view(seq & 1 == 1); // even chirp -> ping, odd -> pong

    let fmt = format();
    let per_rx_out = payload_bytes(samples, 1, fmt);
    let stride = sensor::adc_bytes_per_rx(samples as u16);
    if COUNT.load(Ordering::Acquire) == RING_CHIRPS {
        DROPPED.fetch_add(1, Ordering::Relaxed);
        WAKER.wake();
        return;
    }
    // Regenerate the kept-pair mask for this chirp (shared across the RX).
    let kept = match fmt {
        Format::Sub12(k) => {
            let m = unsafe { &mut *MASK.0.get() };
            mask_fill(seq, samples / 2, k as usize, m)
        }
        Format::Packed12 => 0,
    };
    let head = HEAD.load(Ordering::Relaxed);
    // Safe: single producer; COUNT < RING_CHIRPS means slots[head] is free.
    // The copy runs with interrupts ENABLED so the priority-0 SPI feed ISR
    // can preempt it (a chirp pack takes up to ~70 us; the 20 MHz TX FIFO
    // underruns after ~50 us).
    {
        let slot = unsafe { &mut (*RING.0.get()).slots[head] };
        slot.seq = seq;
        slot.len = rx_count * per_rx_out;
        for rx in 0..rx_count {
            // Volatile word reads out of the ADCBUF window. Read-only: an M4
            // write to this window raises a FEC access event that asserts the
            // RFS firmware. Each word holds a sample pair; 3 bytes out per
            // kept pair.
            let src = (0x5506_0000usize + rx * stride) as *const u32;
            let dst = slot.data[rx * per_rx_out..].as_mut_ptr();
            match fmt {
                Format::Packed12 => unsafe {
                    for w in 0..samples / 2 {
                        let v = core::ptr::read_volatile(src.add(w));
                        pack_pair(v, dst.add(w * 3));
                    }
                },
                Format::Sub12(_) => unsafe {
                    // Only the kept words are even read from the ADCBUF.
                    let m = &*MASK.0.get();
                    for (i, &pi) in m[..kept].iter().enumerate() {
                        let v = core::ptr::read_volatile(src.add(pi as usize));
                        pack_pair(v, dst.add(i * 3));
                    }
                },
            }
        }
    }
    HEAD.store((head + 1) % RING_CHIRPS, Ordering::Relaxed);
    COUNT.fetch_add(1, Ordering::Release);
    WAKER.wake();
}

/// Pack a sample-pair word into 3 output bytes, counting 12-bit overflow.
#[inline(always)]
unsafe fn pack_pair(v: u32, p: *mut u8) {
    let s0 = v as u16;
    let s1 = (v >> 16) as u16;
    if (s0 as i16).wrapping_add(2048) as u16 > 4095
        || (s1 as i16).wrapping_add(2048) as u16 > 4095
    {
        OOR.fetch_add(1, Ordering::Relaxed);
    }
    unsafe {
        p.write(s0 as u8);
        p.add(1).write((((s0 >> 8) & 0xF) | ((s1 & 0xF) << 4)) as u8);
        p.add(2).write((s1 >> 4) as u8);
    }
}
