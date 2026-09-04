use iwrl6432_hal::sensor::{ChirpCommon, ChirpTiming, FrameCfg, RadarConfig, Resolution};

/// A named preset radar configuration.
pub struct Profile {
    pub name: &'static str,
    /// One-line summary for the `profiles` listing.
    pub summary: &'static str,
    /// What it is for, shown by `show <name>`.
    pub detail: &'static str,
    pub cfg: RadarConfig,
}

/// Profile started automatically at boot. Set to `None` to boot idle and
/// wait for a CLI `start`.
pub const AUTO_START: Option<&str> = Some("standard");

/// Build a config from the shared base: coarse resolution, 3 RX, both TX and
/// continuous frames. LSBs are 100 ns for ramp/idle/burst, 25 ns for frame,
/// ~28.61 kHz/us for slope, and 0xBE00 to 0xD555 spans 57 to 64 GHz.
fn cfg(
    samples: u16,
    rate_code: u8,       // f_s = 100/code MSPS
    slope: i16,          // slope LSBs
    ramp: u16,           // ramp_end_time, 100 ns LSB
    idle: u16,           // idle_time, 100 ns LSB
    chirps_per_burst: u16,
    bursts_per_frame: u16,
    burst_period: u32,  // 100 ns LSB
    frame_period: u32,  // 25 ns LSB
) -> RadarConfig {
    // 2.1845 * ramp_us * slope_mhz_us, over min(idle_us minus 1, 6)
    let ramp_us = ramp as f32 / 10.0;
    let slope_mhz = slope as f32 * 0.02861;
    let idle_us = idle as f32 / 10.0;
    let div = if idle_us - 1.0 < 6.0 { idle_us - 1.0 } else { 6.0 };
    let crd = (2.1845 * ramp_us * slope_mhz / div) as u16;
    RadarConfig {
        resolution: Resolution { freq_high_res: false, timing_high_res: false },
        common: ChirpCommon {
            sample_rate_code: rate_code,
            out_bits_sel: 0, // 12-bit round
            fir_sel: 0,      // long FIR
            num_adc_samples: samples,
            // TDMA-2TX, so chirps_per_burst must stay even to hold ping/pong parity
            mimo_pattern: 1,
            ramp_end_time: ramp,
            rx_hpf_sel: 1, // 350 kHz
            hpf_fast_init_duration: 15,
            crd_nslope_mag: crd,
            misc_settings: 0,
        },
        timing: ChirpTiming {
            idle_time: idle,
            adc_skip_samples: 0,
            adc_start_frac: 0,
            tx_start_time: 0,
            freq_slope: slope,
            freq_start: 0xC9AA, // ~60.5 GHz start (factory-cal band center region)
            tx_en: 0x3,
            tx_bpm_en: 0,
        },
        frame: FrameCfg {
            chirps_per_burst,
            bursts_per_frame,
            chirp_accum: 0,
            burst_periodicity: burst_period,
            frame_periodicity: frame_period,
            num_frames: 0,
        },
        capture_format: iwrl6432_hal::capture::Format::Packed12,
    }
}

/// List every preset profile.
pub fn profiles() -> [Profile; 8] {
    [
        Profile {
            name: "standard",
            summary: "packed12 | 512 samp, 1.6 GHz sweep, 9 cm bins, 600 chirps/s",
            detail: "Balanced default: 9 cm range bins AND fine Doppler \
                     (0.21 m/s, 12 ms coherent window) at 600 chirps/s, \
                     ~1.4 MB/s wire (within the ~1.7 MB/s link). 25 fps.",
            // 512 @ 12.5M, 40.0 MHz/us (1398 LSB), 45 us ramp, 2x12 @1 ms, 25 fps
            cfg: cfg(512, 8, 1398, 450, 500, 2, 12, 10_000, 1_600_000),
        },
        Profile {
            name: "motion",
            summary: "packed12 | 128 samp, 18 cm bins, 1600 chirps/s, 50 fps",
            detail: "Short chirps at 500 us burst spacing: the highest chirp \
                     rate and velocity ceiling, for micro-Doppler. Coarse 18 cm \
                     range bins (short sweep), 50 fps refresh.",
            // 128 @ 6.25M (code 16), 40.0 MHz/us, 25 us ramp, 2x16 @0.5 ms, 50 fps
            cfg: cfg(128, 16, 1398, 250, 300, 2, 16, 5_000, 800_000),
        },
        Profile {
            name: "imaging",
            summary: "bfp16 | 1024 samp, 3.3 GHz sweep, 4.6 cm bins",
            detail: "4.6 cm bins from a 3.3 GHz sweep, semi-lossless bfp16 \
                     (~78 dB spectrum PSNR, no reconstruction needed): \
                     1.31 MB/s wire at 400 chirps/s, 25 fps. Bursts at 3 ms \
                     (v_max 0.41 m/s): the 1024-samp ADCBUF drain takes \
                     ~1.1 ms wall time under SPI load, and tighter bursts \
                     lose chirp IRQs / skip bursts (HW-verified; see the \
                     status resync counter).",
            // 1024 @ 12.5M, 40.0 MHz/us, 90 us ramp, 2x8 @3 ms, 25 fps.
            // start ~58.4 GHz so the 3.3 GHz sweep stays in-band
            cfg: {
                let mut c = cfg(1024, 8, 1398, 900, 60, 2, 8, 30_000, 1_600_000);
                c.timing.freq_start = 0xC2AA;
                c.capture_format = iwrl6432_hal::capture::Format::Bfp16;
                c
            },
        },
        Profile {
            name: "hires",
            summary: "sub12 6/16 | 1536 samp, 5 GHz sweep, 3.0 cm bins",
            detail: "3.0 cm bins from a 5 GHz sweep at keep 6/16 (-4.3 dB \
                     SNR), ~1.04 MB/s wire. keep is capped by the RFS, not \
                     the link (~1.7 MB/s): at keep 7 it skips one burst per \
                     frame under streaming load. 400 chirps/s, 25 fps.",
            // 1536 @ 12.5M, 40.7 MHz/us (1422 LSB), 130 us ramp, 2x8 @1 ms, 25 fps.
            // start 58 GHz, so the 5 GHz sweep ends ~63 GHz and stays in-band
            cfg: {
                let mut c = cfg(1536, 8, 1422, 1300, 60, 2, 8, 10_000, 1_600_000);
                c.timing.freq_start = 0xC155;
                c.capture_format = iwrl6432_hal::capture::Format::Sub12(6);
                c
            },
        },
        Profile {
            name: "maxres",
            summary: "sub12 4/16 | 2048 samp, 7 GHz sweep, 2.1 cm bins",
            detail: "Range-resolution floor: 2.1 cm bins from the full 7 GHz \
                     band (57-64 GHz). Max sample count caps keep at 4/16 \
                     (-6 dB SNR). 400 chirps/s, 25 fps.",
            // 2048 @ 12.5M, 42.7 MHz/us (1493 LSB), 170 us ramp, 2x8 @1 ms, 25 fps.
            // start 57 GHz, full-band sweep to 64 GHz
            cfg: {
                let mut c = cfg(2048, 8, 1493, 1700, 60, 2, 8, 10_000, 1_600_000);
                c.timing.freq_start = 0xBE00;
                c.capture_format = iwrl6432_hal::capture::Format::Sub12(4);
                c
            },
        },
        Profile {
            name: "deep_std",
            summary: "packed12 | 512 samp, 9 cm bins, 0.083 m/s v_res, 10 fps",
            detail: "standard slowed to 10 fps: same 9 cm bins and 600 \
                     chirps/s, but 30 bursts per frame give a 30 ms coherent \
                     window - 2.5x finer velocity resolution (0.083 m/s) for \
                     slow micro-motion (bed-scale monitoring). ~1.4 MB/s wire.",
            // 512 @ 12.5M, 40.0 MHz/us, 45 us ramp, 50 us idle, 2x30 @1 ms, 10 fps
            cfg: cfg(512, 8, 1398, 450, 500, 2, 30, 10_000, 4_000_000),
        },
        Profile {
            name: "deep_range",
            summary: "sub12 4/16 | 2048 samp, 7 GHz, 2.1 cm bins, 10 fps",
            detail: "maxres slowed to 10 fps: same 2.1 cm bins, but 24 bursts \
                     per frame give a ~24 ms coherent window and the finest \
                     velocity resolution. 480 chirps/s, keep 4/16 (-6 dB SNR).",
            // 2048 @ 12.5M, 42.7 MHz/us, 170 us ramp, 2x24 @1 ms, 10 fps.
            cfg: {
                let mut c = cfg(2048, 8, 1493, 1700, 60, 2, 24, 10_000, 4_000_000);
                c.timing.freq_start = 0xBE00;
                c.capture_format = iwrl6432_hal::capture::Format::Sub12(4);
                c
            },
        },
        Profile {
            name: "deep_fine",
            summary: "sub12 5/16 | 1536 samp, 5 GHz, 3.0 cm bins, 10 fps",
            detail: "hires slowed to 10 fps: 3.0 cm bins, 24 bursts per frame \
                     for the same fine velocity resolution at better SNR than \
                     deep_range. 480 chirps/s, keep 5/16 (-5 dB SNR).",
            // 1536 @ 12.5M, 40.7 MHz/us, 130 us ramp, 2x24 @1 ms, 10 fps.
            cfg: {
                let mut c = cfg(1536, 8, 1422, 1300, 60, 2, 24, 10_000, 4_000_000);
                c.timing.freq_start = 0xC155;
                c.capture_format = iwrl6432_hal::capture::Format::Sub12(5);
                c
            },
        },
    ]
}

/// Look up a preset profile by name.
pub fn find(name: &str) -> Option<Profile> {
    profiles().into_iter().find(|p| p.name == name)
}

/// Derived metrics for a config, for the CLI `show` view.
pub struct Metrics {
    pub fs_msps: f32,
    pub slope_mhz_us: f32,
    pub sweep_mhz: f32,
    pub range_res_m: f32,
    pub max_range_m: f32,
    pub v_max_ms: f32,
    pub v_res_ms: f32,
    pub chirps_per_sec: u32,
    pub wire_bytes_per_sec: u32,
    pub fps: f32,
}

/// Derive the [`Metrics`] a config implies.
pub fn metrics(c: &RadarConfig) -> Metrics {
    const C_LIGHT: f32 = 3.0e8;
    const LAMBDA_M: f32 = 4.96e-3; // ~60.5 GHz
    let fs = 100.0 / c.common.sample_rate_code as f32; // MSPS
    let slope = c.timing.freq_slope as f32 * 0.02861; // MHz/us
    let sampling_us = c.common.num_adc_samples as f32 / fs;
    let sweep = slope * sampling_us; // MHz
    let range_res = C_LIGHT / (2.0 * sweep * 1.0e6);
    let max_range = (fs / 2.0) * C_LIGHT / (2.0 * slope * 1.0e12) * 1.0e6;
    // Doppler is measured across bursts, so repetition is the burst period
    let t_rep_s = c.frame.burst_periodicity as f32 * 100.0e-9;
    let coherent_s = c.frame.bursts_per_frame as f32 * t_rep_s;
    let per_frame = c.frame.chirps_per_burst as u32 * c.frame.bursts_per_frame as u32;
    let fps = 40.0e6 / c.frame.frame_periodicity as f32;
    let cps = (per_frame as f32 * fps) as u32;
    let payload =
        iwrl6432_hal::capture::payload_bytes(c.common.num_adc_samples as usize, 3, c.capture_format)
            as u32;
    Metrics {
        fs_msps: fs,
        slope_mhz_us: slope,
        sweep_mhz: sweep,
        range_res_m: range_res,
        max_range_m: max_range,
        v_max_ms: LAMBDA_M / (4.0 * t_rep_s),
        v_res_ms: LAMBDA_M / (2.0 * coherent_s),
        chirps_per_sec: cps,
        wire_bytes_per_sec: cps * (payload + 8),
        fps,
    }
}
