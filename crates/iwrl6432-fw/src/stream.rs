// Chunk magics: 5A A5 status header, A5 5C packed12, A5 5B bfp16, A5 6k sub12
// keeping k of 16 pairs, A5 5E config descriptor (sent once a second).

use embassy_time::{Duration, Instant};
use iwrl6432_hal::{capture, mcspi, sensor};

/// Config-descriptor chunk magic, carrying the geometry and RF params the host
/// needs for physical axes and the MIMO layout.
const META_MAGIC: u8 = 0x5E;
/// Descriptor payload length. Mirrored by the host
/// `struct.unpack(">BBBBHHHBBhHHxxIII", ...)`.
const META_LEN: usize = 32;
/// Re-emit the descriptor at least this often even without a reconfigure.
const META_PERIOD: Duration = Duration::from_millis(1000);

/// A push stalled this long means the host stopped clocking mid-batch and never
/// re-polled, so recover to poll-listening.
const STALL_TIMEOUT: Duration = Duration::from_millis(100);

/// No recognized poll for this long with nothing in flight means the slave's
/// word framing desynced, which [`STALL_TIMEOUT`] cannot catch. Sits well above
/// the host's active poll gap, so it only fires on a real wedge or an idle port.
const IDLE_TIMEOUT: Duration = Duration::from_millis(300);

fn chirp_magic() -> u8 {
    match capture::format() {
        capture::Format::Packed12 => 0x5C,
        capture::Format::Sub12(k) => 0x60 | k,
        capture::Format::Bfp16 => 0x5B,
    }
}

/// Write a config-descriptor chunk into `tx` at word `nw`, returning the new
/// word index.
fn build_meta(tx: &mut [u32], nw: usize, cfg: &sensor::RadarConfig) -> usize {
    let (c, t, f) = (&cfg.common, &cfg.timing, &cfg.frame);
    let mut d = [0u8; META_LEN];
    d[0] = 1; // descriptor version
    d[1] = capture::rx_count() as u8;
    d[2] = match capture::format() {
        capture::Format::Packed12 => 0,
        capture::Format::Sub12(k) => k,
        capture::Format::Bfp16 => 16,
    };
    d[3] = c.sample_rate_code;
    d[4..6].copy_from_slice(&(capture::num_samples() as u16).to_be_bytes());
    d[6..8].copy_from_slice(&f.chirps_per_burst.to_be_bytes());
    d[8..10].copy_from_slice(&f.bursts_per_frame.to_be_bytes());
    d[10] = t.tx_en as u8;
    d[11] = c.mimo_pattern;
    d[12..14].copy_from_slice(&t.freq_slope.to_be_bytes());
    d[14..16].copy_from_slice(&c.ramp_end_time.to_be_bytes());
    d[16..18].copy_from_slice(&t.idle_time.to_be_bytes());
    // 18..20 reserved
    d[20..24].copy_from_slice(&t.freq_start.to_be_bytes());
    d[24..28].copy_from_slice(&f.burst_periodicity.to_be_bytes());
    d[28..32].copy_from_slice(&f.frame_periodicity.to_be_bytes());

    tx[nw] = u32::from_be_bytes([0xA5, META_MAGIC, (META_LEN >> 8) as u8, META_LEN as u8]);
    tx[nw + 1] = capture::generation();
    for (k, w) in d.chunks_exact(4).enumerate() {
        tx[nw + 2 + k] = u32::from_be_bytes([w[0], w[1], w[2], w[3]]);
    }
    nw + 2 + META_LEN / 4
}

/// Keep the SPI ISR's batch double buffer fed from the capture ring and
/// watchdog abandoned pushes. Polls are answered by the ISR, so batch building
/// never sits in the poll-response path.
#[embassy_executor::task]
pub async fn spi_stream() {
    let mut buf = [0u8; capture::MAX_CHIRP_BYTES];
    let mut ring_gen = capture::generation();
    let mut meta_deadline = Instant::now();
    let mut meta_gen = ring_gen.wrapping_sub(1); // force the first descriptor
    let mut stall = (0usize, Instant::now());
    let mut idle = (mcspi::polls(), Instant::now());
    loop {
        // a reconfigure resets the ring, so drop batches armed for the old profile
        let g = capture::generation();
        if g != ring_gen {
            ring_gen = g;
            mcspi::flush_armed();
        }

        // the deadline keeps moving while a push makes progress
        let (pos, len) = mcspi::push_progress();
        let now = Instant::now();
        if pos == 0 || pos >= len || pos != stall.0 {
            stall = (pos, now);
        } else if now > stall.1 + STALL_TIMEOUT {
            mcspi::force_idle();
            stall = (0, now);
        }

        // re-init the channel so a framing-desynced port self-heals without a reset
        let p = mcspi::polls();
        if p != idle.0 {
            idle = (p, now);
        } else if !mcspi::is_serving() && now > idle.1 + IDLE_TIMEOUT {
            mcspi::force_idle();
            idle = (p, now);
        }

        if capture::pending() == 0 {
            embassy_futures::yield_now().await;
            continue;
        }
        // build as late as possible so batches carry a full host cycle of chirps,
        // since an armed batch cannot grow and tiny reads waste the host's fixed
        // per-poll USB cost
        let (pos, len) = mcspi::push_progress();
        let reading = mcspi::is_serving();
        let hungry = !reading && !mcspi::any_armed();
        let near_done = reading && pos * 4 >= len * 3;
        if !(hungry || near_done || capture::nearly_full()) {
            embassy_futures::yield_now().await;
            continue;
        }
        let Some(tx) = mcspi::build_slot() else {
            embassy_futures::yield_now().await;
            continue;
        };

        let mut nw = 2;
        let mut first_seq = 0;
        let mut total = 0usize;

        if let Some(cfg) = sensor::active_config() {
            if g != meta_gen || now >= meta_deadline {
                nw = build_meta(tx, nw, &cfg);
                total += 8 + META_LEN;
                meta_gen = g;
                meta_deadline = now + META_PERIOD;
            }
        }

        let magic = chirp_magic();
        for i in 0.. {
            let Some(c) = capture::pop_chirp(&mut buf) else { break };
            if i == 0 {
                first_seq = c.seq;
            }
            tx[nw] = u32::from_be_bytes([0xA5, magic, (c.len >> 8) as u8, c.len as u8]);
            tx[nw + 1] = c.seq;
            for (k, ch) in buf[..c.len].chunks_exact(4).enumerate() {
                tx[nw + 2 + k] = u32::from_be_bytes([ch[0], ch[1], ch[2], ch[3]]);
            }
            nw += 2 + c.len / 4;
            total += 8 + c.len;
            if nw + 2 + c.len / 4 > mcspi::TX_BUF_WORDS {
                break;
            }
        }
        tx[0] = u32::from_be_bytes([0x5A, 0xA5, (total >> 8) as u8, total as u8]);
        tx[1] = first_seq;
        mcspi::arm(nw);
    }
}
