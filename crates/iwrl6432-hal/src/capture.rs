use core::sync::atomic::{AtomicU32, AtomicU8, AtomicUsize, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::sensor;

/// On-device sample encoding applied in the drain ISR.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// Baseline format, packing 2 samples into 3 bytes of 12-bit two's
    /// complement. Out-of-range samples alias and are counted by
    /// [`out_of_range`].
    Packed12,
    /// Random subsampling for host-side compressed-sensing reconstruction,
    /// keeping `k` of every 16 sample pairs, Packed12-encoded. Wire rate
    /// scales by k/16 and the reconstruction loses `10*log10(16/k)` dB SNR.
    Sub12(u8),
    /// Block floating point, 1.41x smaller than Packed12. Each 16-sample block
    /// is a shared 0..4 shift exponent byte plus 16 int8 mantissas. Exact when
    /// the block max is under 128, otherwise quantization sits ~48 dB below the
    /// block peak.
    Bfp16,
}

/// Mask granularity for [`Format::Sub12`]. Pairs are drawn per block of this
/// many, which stratifies the draw so kept samples never cluster badly.
pub const SUB12_BLOCK_PAIRS: usize = 16;

/// Ring capacity in chirps, 62 KB of the 128 KB APPSS RAM3 shared with the SPI
/// batch buffers and the stack. Must absorb a whole frame's chirp clump, since
/// the link only drains in the gaps between bursts.
const RING_CHIRPS: usize = 19;
/// Per-chirp storage, which [`init`] asserts the configured geometry fits.
/// Consumers size their pop buffers with this.
pub const MAX_CHIRP_BYTES: usize = 3264;

// lock-free SPSC: the chirp ISR is the only producer, the streaming task the
// only consumer. no critical section is held during the pack, since holding
// PRIMASK that long underruns the 20 MHz TX FIFO
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
// ISR timing in 40 MHz RTI ticks, since this M4 has no DWT CYCCNT. QUEUED
// counts entries within 10 us of the previous exit, a backlog that can run the
// ADCBUF read into the next overwrite
static PACK_MAX_CYC: AtomicU32 = AtomicU32::new(0);
static QUEUED: AtomicU32 = AtomicU32::new(0);
static LAST_EXIT_CYC: AtomicU32 = AtomicU32::new(0);
// chirps dropped because the DFE rewrote the ADCBUF half mid-read. shipping one
// poisons the frame's Doppler with noise that grows along fast time
static TORN: AtomicU32 = AtomicU32::new(0);
// OFFSET is the self-calibrated hardware-index-to-seq offset
static RESYNC: AtomicU32 = AtomicU32::new(0);
static OFFSET: AtomicU32 = AtomicU32::new(0);
static OFFSET_VALID: AtomicU8 = AtomicU8::new(0);
static CPB: AtomicUsize = AtomicUsize::new(0);
static BPF: AtomicUsize = AtomicUsize::new(0);

// FORMAT codes: 1..15 are Sub12(k), 16 is Packed12, 17 is Bfp16
const FORMAT_BFP16: u8 = SUB12_BLOCK_PAIRS as u8 + 1;
static NUM_SAMPLES: AtomicUsize = AtomicUsize::new(0);
static RX_COUNT: AtomicUsize = AtomicUsize::new(0);
static FORMAT: AtomicU8 = AtomicU8::new(SUB12_BLOCK_PAIRS as u8);

// sized for the most pairs a slot can hold, at 3 bytes per pair
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
    /// Monotonic chirp index since [`init`]. Drops show as gaps host-side.
    pub seq: u32,
    pub len: usize,
}

/// Record the capture geometry and unmask the chirp-avail interrupt. Call after
/// [`sensor::adcbuf_config`] and before [`sensor::start`]. `chirps_per_burst`
/// must be even, since ping/pong is selected by chirp parity.
pub fn init(
    num_adc_samples: u16,
    rx_count: usize,
    format: Format,
    chirps_per_burst: u16,
    bursts_per_frame: u16,
) {
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
        Format::Bfp16 => {
            // 64 gives whole 16-sample blocks and a word-sized per-RX payload
            assert!(num_adc_samples % 64 == 0, "Bfp16 needs samples % 64 == 0");
            FORMAT_BFP16
        }
    };
    NUM_SAMPLES.store(num_adc_samples as usize, Ordering::Relaxed);
    RX_COUNT.store(rx_count, Ordering::Relaxed);
    FORMAT.store(keep, Ordering::Relaxed);
    SEEN.store(0, Ordering::Relaxed);
    DROPPED.store(0, Ordering::Relaxed);
    OOR.store(0, Ordering::Relaxed);
    PACK_MAX_CYC.store(0, Ordering::Relaxed);
    QUEUED.store(0, Ordering::Relaxed);
    TORN.store(0, Ordering::Relaxed);
    RESYNC.store(0, Ordering::Relaxed);
    OFFSET_VALID.store(0, Ordering::Relaxed);
    CPB.store(chirps_per_burst as usize, Ordering::Relaxed);
    BPF.store(bursts_per_frame as usize, Ordering::Relaxed);
    GENERATION.fetch_add(1, Ordering::Relaxed);
    // quiesce the producer while the indexes reset
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

/// Report `(chirps seen, chirps dropped)` since [`init`].
pub fn stats() -> (u32, u32) {
    (SEEN.load(Ordering::Relaxed), DROPPED.load(Ordering::Relaxed))
}

/// Count samples that did not fit 12 bits during [`Format::Packed12`] packing.
/// Nonzero means the stream is corrupt, since the ADC is 12-bit.
pub fn out_of_range() -> u32 {
    OOR.load(Ordering::Relaxed)
}

/// Report `(max ISR duration in 40 MHz ticks, queued entries)` since [`init`].
pub fn isr_timing() -> (u32, u32) {
    (
        PACK_MAX_CYC.load(Ordering::Relaxed),
        QUEUED.load(Ordering::Relaxed),
    )
}

/// Count chirps dropped by the torn-read guard since [`init`].
pub fn torn() -> u32 {
    TORN.load(Ordering::Relaxed)
}

/// Count hardware seq-resync corrections since [`init`].
pub fn resyncs() -> u32 {
    RESYNC.load(Ordering::Relaxed)
}

/// Read the generation counter, bumped by every [`init`], which consumers use
/// to discard batches built for a previous configuration.
pub fn generation() -> u32 {
    GENERATION.load(Ordering::Relaxed)
}

/// Read the number of enabled RX regions per chirp.
pub fn rx_count() -> usize {
    RX_COUNT.load(Ordering::Relaxed)
}

/// Read the ADC samples per chirp per RX.
pub fn num_samples() -> usize {
    NUM_SAMPLES.load(Ordering::Relaxed)
}

/// Read the encoding chirps are stored and streamed in.
pub fn format() -> Format {
    match FORMAT.load(Ordering::Relaxed) {
        FORMAT_BFP16 => Format::Bfp16,
        k if (k as usize) < SUB12_BLOCK_PAIRS => Format::Sub12(k),
        _ => Format::Packed12,
    }
}

/// Compute the payload bytes per chirp for a geometry and format.
pub fn payload_bytes(samples: usize, rx_count: usize, format: Format) -> usize {
    match format {
        Format::Packed12 => rx_count * samples * 3 / 2,
        Format::Sub12(k) => rx_count * (samples / 2 / SUB12_BLOCK_PAIRS) * k as usize * 3,
        Format::Bfp16 => rx_count * (samples / 16) * 17,
    }
}

/// Fill `out` with the kept-pair indices for chirp `seq`, drawn per 16-pair
/// block by a partial Fisher-Yates over an xorshift32 seeded from `seq` alone
/// and emitted ascending. The host mirrors this exactly in
/// `tools/spi_capture.py`, so change both or neither.
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

/// Await the next chirp and copy it into `buf`, which must be at least
/// [`chirp_len`] bytes.
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

/// Pop the oldest chirp into `buf` without blocking, or `None` when empty.
pub fn pop_chirp(buf: &mut [u8]) -> Option<Chirp> {
    if COUNT.load(Ordering::Acquire) == 0 {
        return None;
    }
    let tail = TAIL.load(Ordering::Relaxed);
    // sound because the producer never touches slots[tail] while COUNT > 0
    let slot = unsafe { &(*RING.0.get()).slots[tail] };
    let (seq, len) = (slot.seq, slot.len);
    buf[..len].copy_from_slice(&slot.data[..len]);
    TAIL.store((tail + 1) % RING_CHIRPS, Ordering::Relaxed);
    COUNT.fetch_sub(1, Ordering::Release);
    Some(Chirp { seq, len })
}

/// Count chirps currently buffered.
pub fn pending() -> usize {
    COUNT.load(Ordering::Acquire)
}

/// Report whether the ring is full enough that the consumer should drain now to
/// avoid drops.
pub fn nearly_full() -> bool {
    COUNT.load(Ordering::Acquire) >= RING_CHIRPS - 4
}

/// Compute the payload bytes per chirp for the configured geometry and format.
pub fn chirp_len() -> usize {
    payload_bytes(
        NUM_SAMPLES.load(Ordering::Relaxed),
        RX_COUNT.load(Ordering::Relaxed),
        format(),
    )
}

/// Read the sequence number of the oldest buffered chirp, 0 when empty.
pub fn next_seq() -> u32 {
    if COUNT.load(Ordering::Acquire) == 0 {
        return 0;
    }
    unsafe { (*RING.0.get()).slots[TAIL.load(Ordering::Relaxed)].seq }
}

/// Drain the just-finished ping/pong half into the ring, packing per the
/// configured [`Format`]. If the ring is full the chirp is dropped and counted.
#[unsafe(no_mangle)]
extern "C" fn MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN() {
    let entry = crate::time::ticks_40mhz();
    let prompt = entry.wrapping_sub(LAST_EXIT_CYC.load(Ordering::Relaxed)) >= 400;
    if !prompt {
        QUEUED.fetch_add(1, Ordering::Relaxed);
    }
    let mut seq = SEEN.fetch_add(1, Ordering::Relaxed);
    let samples = NUM_SAMPLES.load(Ordering::Relaxed);
    let rx_count = RX_COUNT.load(Ordering::Relaxed);
    if samples == 0 {
        return;
    }
    // the NVIC holds only one pended chirp IRQ, so an ISR stretched past two
    // chirp completions loses one and seq falls behind, inverting ping/pong
    // parity and TX labels. the hardware counters give the true absolute index,
    // sampled only on prompt burst-last entries where all three are frozen
    let cpb = CPB.load(Ordering::Relaxed);
    if prompt && cpb > 0 {
        let cc = sensor::chirp_count();
        if cc as usize == cpb - 1 {
            let bpf = BPF.load(Ordering::Relaxed);
            let raw = (sensor::frame_count() as u32)
                .wrapping_mul(bpf as u32)
                .wrapping_add(sensor::burst_count() as u32)
                .wrapping_mul(cpb as u32)
                .wrapping_add(cc);
            if OFFSET_VALID.load(Ordering::Relaxed) == 0 {
                // first burst after init, so seq is trustworthy and the offset
                // absorbs whatever epoch the counters start at
                OFFSET.store(seq.wrapping_sub(raw), Ordering::Relaxed);
                OFFSET_VALID.store(1, Ordering::Relaxed);
            } else {
                let true_seq = raw.wrapping_add(OFFSET.load(Ordering::Relaxed));
                let d = true_seq.wrapping_sub(seq);
                if (1..1000).contains(&d) {
                    // lost d chirp IRQs behind us, so relabel and resync
                    RESYNC.fetch_add(1, Ordering::Relaxed);
                    seq = true_seq;
                    SEEN.store(true_seq.wrapping_add(1), Ordering::Relaxed);
                } else if d != 0 {
                    // counter epoch changed, so relearn rather than mislabel
                    OFFSET.store(seq.wrapping_sub(raw), Ordering::Relaxed);
                }
            }
        }
    }
    sensor::adcbuf_view(seq & 1 == 1); // even chirps are ping, odd are pong

    let fmt = format();
    let per_rx_out = payload_bytes(samples, 1, fmt);
    let stride = sensor::adc_bytes_per_rx(samples as u16);
    if COUNT.load(Ordering::Acquire) == RING_CHIRPS {
        DROPPED.fetch_add(1, Ordering::Relaxed);
        WAKER.wake();
        return;
    }
    // the kept-pair mask is shared across the RX
    let kept = match fmt {
        Format::Sub12(k) => {
            let m = unsafe { &mut *MASK.0.get() };
            mask_fill(seq, samples / 2, k as usize, m)
        }
        Format::Packed12 | Format::Bfp16 => 0,
    };
    let head = HEAD.load(Ordering::Relaxed);
    // the pack runs with interrupts enabled so the priority-0 SPI feed ISR can
    // preempt it, since a pack takes up to ~70 us and the TX FIFO underruns
    // after ~50 us. the DFE refills this half from word 0, so a word 0 that
    // changes across the pack means the read overlapped the rewrite
    let guard = unsafe { core::ptr::read_volatile(0x5506_0000 as *const u32) };
    {
        let slot = unsafe { &mut (*RING.0.get()).slots[head] };
        slot.seq = seq;
        slot.len = rx_count * per_rx_out;
        for rx in 0..rx_count {
            // read-only: an M4 write to this window raises a FEC access event
            // that asserts the RFS firmware
            let src = (0x5506_0000usize + rx * stride) as *const u32;
            let dst = slot.data[rx * per_rx_out..].as_mut_ptr();
            match fmt {
                Format::Packed12 => unsafe {
                    // the ADCBUF sits behind the FEC bridge at ~70 cycles per
                    // single word read, so block loads amortize the latency
                    for b in 0..samples / 16 {
                        let w = core::ptr::read_volatile(src.add(b * 8) as *const [u32; 8]);
                        for (i, &v) in w.iter().enumerate() {
                            pack_pair(v, dst.add((b * 8 + i) * 3));
                        }
                    }
                },
                Format::Sub12(_) => unsafe {
                    // only the kept words are read from the ADCBUF at all
                    let m = &*MASK.0.get();
                    for (i, &pi) in m[..kept].iter().enumerate() {
                        let v = core::ptr::read_volatile(src.add(pi as usize));
                        pack_pair(v, dst.add(i * 3));
                    }
                },
                Format::Bfp16 => unsafe {
                    for b in 0..samples / 16 {
                        pack_block_bfp(src.add(b * 8), dst.add(b * 17));
                    }
                },
            }
        }
    }
    if unsafe { core::ptr::read_volatile(0x5506_0000 as *const u32) } != guard {
        TORN.fetch_add(1, Ordering::Relaxed); // dropped, so the host sees a seq gap
    } else {
        HEAD.store((head + 1) % RING_CHIRPS, Ordering::Relaxed);
        COUNT.fetch_add(1, Ordering::Release);
        WAKER.wake();
    }
    let exit = crate::time::ticks_40mhz();
    LAST_EXIT_CYC.store(exit, Ordering::Relaxed);
    PACK_MAX_CYC.fetch_max(exit.wrapping_sub(entry), Ordering::Relaxed);
}

/// Bfp16-encode one 16-sample block into 17 output bytes, a shared shift
/// exponent from the block max plus 16 round-to-nearest int8 mantissas. The
/// words are copied locally first so the slow ADCBUF bus is read exactly once.
/// The host mirrors this bit-exact in `tools/spi_capture.py`.
#[inline(always)]
unsafe fn pack_block_bfp(src: *const u32, dst: *mut u8) {
    let w = unsafe { core::ptr::read_volatile(src as *const [u32; 8]) };
    let mut maxabs = 0u32;
    for &v in &w {
        maxabs |= (v as u16 as i16 as i32).unsigned_abs();
        maxabs |= ((v >> 16) as u16 as i16 as i32).unsigned_abs();
    }
    let exp = (32 - maxabs.leading_zeros()).saturating_sub(7).min(4);
    let half = if exp > 0 { 1i32 << (exp - 1) } else { 0 };
    unsafe {
        dst.write(exp as u8);
        let mut p = dst.add(1);
        for v in w {
            let s0 = v as u16 as i16 as i32;
            let s1 = (v >> 16) as u16 as i16 as i32;
            p.write(((s0 + half) >> exp).clamp(-128, 127) as u8);
            p.add(1).write(((s1 + half) >> exp).clamp(-128, 127) as u8);
            p = p.add(2);
        }
    }
}

/// Pack one sample-pair word into 3 output bytes, counting 12-bit overflow.
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
