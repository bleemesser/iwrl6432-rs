use cortex_m::asm;

// the critical section keeps the read/write from tearing against the stream task
static ACTIVE_CONFIG: critical_section::Mutex<core::cell::Cell<Option<RadarConfig>>> =
    critical_section::Mutex::new(core::cell::Cell::new(None));

/// The config most recently programmed by [`configure`], or `None` if the
/// radar has not been configured this boot.
pub fn active_config() -> Option<RadarConfig> {
    critical_section::with(|cs| ACTIVE_CONFIG.borrow(cs).get())
}

const CT: usize = 0x5100_0000; // chirp timer
const DFE_RX_CFG: usize = 0x5102_0004; // DFE RX config (sample rate/count/bits/FIR)
const FT: usize = 0x5B00_0000; // frame timer
const CT_TRIG_MUX: usize = 0x5A02_0190; // chirp-timer trigger-source mux (HW-trig)
const APPSS_FT_RESET: usize = 0x5604_0088;
const FT_RESET_MASK: u32 = 0x01C0; // M_FE_SENS_FT_RESET_REG_MASK (bits 6-8)
const RFS_CHIRP_MIRROR: usize = 0x2120_01A4; // T_FE_API_RFS_CHIRP_PROFILE_CMD (28 B)
const RFS_SENS_START_MIRROR: usize = 0x2120_01D0; // T_FE_API_RFS_LIVE_MON_SENS_START_CMD
const RFS_SENS_STATUS: usize = 0x2120_01D4; // T_FE_API_RFS_SENSOR_STATUS (32 B)

// Chirp-timer registers (reg_chirptimer.h).
const CT_TRIG: usize = CT + 0x04;
const CT_NUM_CHIRPS_BURSTS: usize = CT + 0x08; // [15:0] chirps/burst, [31:16] bursts/frame
const CT_RESOLUTION: usize = CT + 0x0C; // [0] freq high-res, [1] timing high-res
const CT_FREQ_START: usize = CT + 0x10;
const CT_FREQ_SLOPE: usize = CT + 0x14;
const CT_IDLE_TIME: usize = CT + 0x18;
const CT_RAMP_END_TIME: usize = CT + 0x1C;
const CT_ADC_START_TIME: usize = CT + 0x20; // [9:0] frac, [15:10] skip samples
const CT_TX_START_TIME: usize = CT + 0x24;
const CT_TX_EN_BPM: usize = CT + 0x28; // [3:0] TxEn, [7:4] TxBpm
const CT_TX_MIMO_PATTERN: usize = CT + 0x30;
const CT_BURST_PERIODICITY: usize = CT + 0x34;
const CT_CHIRP_ACCUM: usize = CT + 0x3C;
const CT_MISC_SETTINGS: usize = CT + 0x40; // [3] HpfFastInitEn
const CT_CRD_1: usize = CT + 0x44; // [0] CrdEn, [1] CrdDitherEn
const CT_CRD_2: usize = CT + 0x48; // [14:0] CrdNslopeMag
const CT_HPF_FAST_INIT: usize = CT + 0x88; // [15:8] HpfFastInitPulseWidth
const CT_CHIRP_CNT: usize = CT + 0x5C;
const CT_BURST_CNT: usize = CT + 0x64;
const CT_TRIG_STOP_DONE: usize = CT + 0xD4;

// Frame-timer registers (reg_frametimer.h).
const FT_NUM_FRAMES: usize = FT + 0x04;
const FT_FRAME_PERIODICITY: usize = FT + 0x08;
const FT_TRIGGERING: usize = FT + 0x0C; // [1:0] mode, [5:3] start key, [8:6] stop key
const FT_TRIG_TIMER_VAL: usize = FT + 0x10;
const FT_EVENT0_TIME: usize = FT + 0x18;
const FT_EVENT1_TIME: usize = FT + 0x1C;
const FT_FRAME_CNT: usize = FT + 0x28;
const FT_TRIG_STOP_DONE: usize = FT + 0x2C; // [0] start honored, [1] stop honored

const FT_START_KEY: u32 = 0x7 << 3;
const FT_STOP_KEY: u32 = 0x7 << 6;
const TRIG_STOP_CLEAR: u32 = 0x3;

// ADCBUF.
const ADCBUF_RD: usize = 0x5506_0000; // read window (samples land here)
const ADCBUF_CTRL: usize = 0x5608_0000;
const ADCBUFCFG1: usize = ADCBUF_CTRL + 0x08; // RXnEN + ping/pong view
const ADCBUFCFG2: usize = ADCBUF_CTRL + 0x10; // ADDRX0[10:0], ADDRX1[26:16]
const ADCBUFCFG3: usize = ADCBUF_CTRL + 0x14; // ADDRX2[10:0], ADDRX3[26:16]
const ADCBUFCFG1_RESET: u32 = 0x0001_0000; // ADCBUFPIPOSEL=1 (auto ping-pong)
const ADCBUFCFG1_PIPOOVRCNT: u32 = 1 << 10; // read-window view (1=ping, 0=pong)

#[inline]
fn rd(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
#[inline]
fn wr(addr: usize, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) };
}
#[inline]
fn rmw(addr: usize, clear: u32, set: u32) {
    wr(addr, (rd(addr) & !clear) | set);
}

/// Timing and frequency resolution, setting the LSB of every timing field and
/// the width of the frequency words. [`FrameCfg::frame_periodicity`] is always
/// in XTAL clocks regardless.
#[derive(Clone, Copy)]
pub struct Resolution {
    /// Frequency word width: false = 16-bit, true = 24-bit.
    pub freq_high_res: bool,
    /// Timing LSB: false = 100 ns, true = 20 ns.
    pub timing_high_res: bool,
}

/// Chirp profile parameters shared across the sweep.
#[derive(Clone, Copy)]
pub struct ChirpCommon {
    /// ADC sample-rate code, where MSPS is 400 / (code * 4). Range 8..=100.
    pub sample_rate_code: u8,
    /// Output bits: 0 = 12-bit round-to-4-LSB ... 5 = 16-bit.
    pub out_bits_sel: u8,
    /// DFE FIR: 0 = long (90% visibility), 1 = short (80%).
    pub fir_sel: u8,
    /// ADC samples per chirp per RX. Range 2..=2048.
    pub num_adc_samples: u16,
    /// TX MIMO pattern: 0 = disabled (use [`ChirpTiming::tx_en`]), 1 = TDMA-2TX, 4 = BPM-2TX.
    pub mimo_pattern: u8,
    /// FMCW ramp duration. LSB = timing resolution.
    pub ramp_end_time: u16,
    /// RX HPF corner: 0 = 175 kHz, 1 = 350, 2 = 700, 3 = 1400.
    pub rx_hpf_sel: u8,
    /// HPF fast-init duration, LSB 100 ns. Range 5..=100.
    pub hpf_fast_init_duration: u8,
    /// Controlled-ramp-down slope magnitude (CrdNslopeMag[14:0], LSB ~457 kHz/us).
    pub crd_nslope_mag: u16,
    /// bit0 HPF-fast-init DIS, bit1 SYNTH-CRD DIS, bit2 CRD-dither DIS,
    /// bit3 PA-blanking EN. 0 = all features on, PA-blanking off.
    pub misc_settings: u8,
}

/// Chirp profile timing and frequency parameters.
#[derive(Clone, Copy)]
pub struct ChirpTiming {
    /// Idle time before the ramp. LSB = timing resolution.
    pub idle_time: u16,
    /// ADC samples to skip at ramp start. Range 0..=63.
    pub adc_skip_samples: u8,
    /// Fractional ADC start time (ADC_START_TIME[9:0]).
    pub adc_start_frac: u16,
    /// TX start time, signed. LSB 20 ns.
    pub tx_start_time: i16,
    /// FMCW slope, signed. LSB ~28.61 kHz/us (0x4D ~ 2.2 MHz/us).
    pub freq_slope: i16,
    /// RF start frequency, 0xBE00 to 0xD555 spanning 57 to 64 GHz at 16-bit
    /// resolution, 0x00BE0000 to 0x00D55555 at 24-bit. Sweep must stay in-band.
    pub freq_start: u32,
    /// Per-chirp TX enable mask (bit0 TX0, bit1 TX1).
    pub tx_en: u16,
    /// Per-chirp TX BPM enable mask.
    pub tx_bpm_en: u16,
}

/// Frame configuration, `bursts_per_frame` bursts of `chirps_per_burst` chirps.
#[derive(Clone, Copy)]
pub struct FrameCfg {
    /// Range 1..=65535. Must be even when using [`crate::capture`], for parity.
    pub chirps_per_burst: u16,
    /// Range 1..=4096.
    pub bursts_per_frame: u16,
    /// Chirp accumulation: 0 = off, else 2..=64.
    pub chirp_accum: u8,
    /// Time between burst starts. LSB = timing resolution.
    pub burst_periodicity: u32,
    /// Time between frame starts. LSB = 1 XTAL clock (25 ns at 40 MHz).
    pub frame_periodicity: u32,
    /// Frames to run, 0 for continuous. 1 is accepted but never starts a frame.
    pub num_frames: u16,
}

/// Complete radar configuration.
#[derive(Clone, Copy)]
pub struct RadarConfig {
    pub resolution: Resolution,
    pub common: ChirpCommon,
    pub timing: ChirpTiming,
    pub frame: FrameCfg,
    /// Sample encoding applied by the capture drain, not an RFS parameter.
    pub capture_format: crate::capture::Format,
}

impl Default for RadarConfig {
    /// Build a known-good streaming profile: ~60.5 GHz start, 2.2 MHz/us over a
    /// 40 us ramp, 12.5 MSPS, 256 samples, both TX, 800 chirps/s at 25 fps.
    fn default() -> Self {
        RadarConfig {
            resolution: Resolution {
                // the RFS has only ever seen RES_00 encoding of these registers
                freq_high_res: false,
                timing_high_res: false,
            },
            common: ChirpCommon {
                sample_rate_code: 8, // 12.5 MSPS
                out_bits_sel: 0,     // 12-bit round
                fir_sel: 0,          // long FIR
                num_adc_samples: 256,
                // TDMA-2TX like every preset, since the host DSP needs the
                // 6-channel virtual array the streamed metadata promises
                mimo_pattern: 1,
                ramp_end_time: 400, // 40 us
                rx_hpf_sel: 1,      // 350 kHz
                hpf_fast_init_duration: 15, // 1.5 us
                // 2.1845 * ramp_us * slope_mhz_us, over min(idle_us minus 1, 6)
                crd_nslope_mag: 38,
                misc_settings: 0,
            },
            timing: ChirpTiming {
                idle_time: 60, // 6 us
                adc_skip_samples: 0,
                adc_start_frac: 0,
                tx_start_time: 0,
                freq_slope: 0x4D,   // ~2.2 MHz/us (== factory-cal slope)
                freq_start: 0xC9AA, // ~60.5 GHz start frequency
                tx_en: 0x3,
                tx_bpm_en: 0,
            },
            frame: FrameCfg {
                chirps_per_burst: 2,
                bursts_per_frame: 16,
                chirp_accum: 0,
                burst_periodicity: 10_000,    // 1 ms (> 2 * (idle + ramp))
                frame_periodicity: 1_600_000, // 40 ms -> 25 fps
                num_frames: 0,
            },
            capture_format: crate::capture::Format::Packed12,
        }
    }
}

/// The [`RadarConfig`] field [`configure`] rejected.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigError {
    NumAdcSamples,
    SampleRateCode,
    AdcSkipSamples,
    FrameCounts,
}

/// Frame trigger mode for [`start`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FrameTrigMode {
    /// Frames start as soon as [`start`] fires.
    SwImmediate = 0,
    /// First frame after FRAME_TRIG_TIMER_VAL.
    SwTimer = 1,
    /// External SYNC_IN, low-power.
    HwLowPower = 2,
    /// External SYNC_IN, low-jitter.
    HwLowJitter = 3,
}

/// Program the chirp timer, DFE and frame timer from `cfg`. Does not start
/// frames, which [`start`] does. Nothing is written if a field is rejected.
pub fn configure(cfg: &RadarConfig) -> Result<(), ConfigError> {
    let (c, t, f) = (&cfg.common, &cfg.timing, &cfg.frame);
    if !(2..=2048).contains(&c.num_adc_samples) {
        return Err(ConfigError::NumAdcSamples);
    }
    if !(8..=100).contains(&c.sample_rate_code) {
        return Err(ConfigError::SampleRateCode);
    }
    if t.adc_skip_samples > 63 {
        return Err(ConfigError::AdcSkipSamples);
    }
    if f.chirps_per_burst == 0 || f.bursts_per_frame == 0 {
        return Err(ConfigError::FrameCounts);
    }

    // idle the frame timer so re-arming works if a previous run left it running
    reset();

    // resolution first, since it defines the units of everything below
    wr(
        CT_RESOLUTION,
        cfg.resolution.freq_high_res as u32 | ((cfg.resolution.timing_high_res as u32) << 1),
    );

    // sample rate[11:4], samples[23:12], FIR[25:24], bits[28:26]
    rmw(
        DFE_RX_CFG,
        0x1FFF_FFF0,
        ((c.sample_rate_code as u32) << 4)
            | ((c.num_adc_samples as u32) << 12)
            | ((c.fir_sel as u32) << 24)
            | ((c.out_bits_sel as u32) << 26),
    );
    wr(CT_TX_MIMO_PATTERN, c.mimo_pattern as u32);
    wr(CT_RAMP_END_TIME, c.ramp_end_time as u32);
    rmw(CT_HPF_FAST_INIT, 0xFF00, (c.hpf_fast_init_duration as u32) << 8);
    let hpf_en = (c.misc_settings & 0x1) == 0;
    rmw(CT_MISC_SETTINGS, 1 << 3, (hpf_en as u32) << 3);
    rmw(CT_CRD_2, 0x7FFF, (c.crd_nslope_mag & 0x7FFF) as u32);
    let crd_en = (c.misc_settings & 0x2) == 0;
    let crd_dither_en = (c.misc_settings & 0x4) == 0;
    rmw(CT_CRD_1, 0x3, crd_en as u32 | ((crd_dither_en as u32) << 1));
    // plain-RAM mirror the RFS reads for pre-frame analog setup
    wr(
        RFS_CHIRP_MIRROR,
        c.sample_rate_code as u32 | ((c.rx_hpf_sel as u32) << 16),
    );
    for i in 1..7 {
        wr(RFS_CHIRP_MIRROR + i * 4, 0);
    }

    wr(CT_IDLE_TIME, t.idle_time as u32);
    wr(
        CT_ADC_START_TIME,
        ((t.adc_skip_samples as u32) << 10) | (t.adc_start_frac as u32 & 0x3FF),
    );
    wr(CT_TX_START_TIME, t.tx_start_time as u16 as u32);
    wr(CT_FREQ_SLOPE, t.freq_slope as u16 as u32);
    wr(CT_FREQ_START, t.freq_start);
    wr(
        CT_TX_EN_BPM,
        (t.tx_en as u32 & 0xF) | ((t.tx_bpm_en as u32 & 0xF) << 4),
    );

    wr(
        CT_NUM_CHIRPS_BURSTS,
        f.chirps_per_burst as u32 | ((f.bursts_per_frame as u32) << 16),
    );
    wr(CT_CHIRP_ACCUM, f.chirp_accum as u32);
    wr(CT_BURST_PERIODICITY, f.burst_periodicity);
    wr(FT_FRAME_PERIODICITY, f.frame_periodicity);
    wr(FT_NUM_FRAMES, f.num_frames as u32);
    wr(FT_EVENT0_TIME, 0); // GPIO frame-sync events off
    wr(FT_EVENT1_TIME, 0);

    critical_section::with(|cs| ACTIVE_CONFIG.borrow(cs).set(Some(*cfg)));
    Ok(())
}

/// Arm and start frames. Call after [`configure`].
pub fn start(mode: FrameTrigMode) {
    // re-assert the DFE clock un-gate, since the RFS reprograms that register
    // during boot and a gated DFE writes nothing to the ADCBUF
    wr(super::fecss::FEC_IPCFGCLKGATE1_PUB, 0x7007);

    // plain RAM, no doorbell. 0 disables the live monitors
    for i in 0..8 {
        wr(RFS_SENS_STATUS + i * 4, 0);
    }
    wr(RFS_SENS_START_MIRROR, 0);
    wr(FT_TRIG_TIMER_VAL, 0);
    wr(FT_TRIG_STOP_DONE, TRIG_STOP_CLEAR);
    wr(CT_TRIG_STOP_DONE, TRIG_STOP_CLEAR);
    wr(CT_TRIG, 0); // chirp timer is triggered by the frame timer
    wr(CT_TRIG_MUX, matches!(mode, FrameTrigMode::HwLowPower | FrameTrigMode::HwLowJitter) as u32);

    let m = mode as u32 & 0x3;
    wr(FT_TRIGGERING, m);
    wr(FT_TRIGGERING, m | FT_START_KEY); // fire
}

/// Stop frames at the next frame boundary.
pub fn stop() {
    rmw(FT_TRIGGERING, FT_STOP_KEY, FT_STOP_KEY);
}

/// Force the frame timer back to idle. This resets the FT registers, which
/// [`configure`] reprograms.
pub fn reset() {
    wr(FT_TRIG_STOP_DONE, TRIG_STOP_CLEAR);
    wr(CT_TRIG_STOP_DONE, TRIG_STOP_CLEAR);
    rmw(APPSS_FT_RESET, 0, FT_RESET_MASK);
    asm::delay(2_000); // ~10 us hold
    rmw(APPSS_FT_RESET, FT_RESET_MASK, 0);
}

/// Report whether the frame timer has honored the start.
pub fn start_honored() -> bool {
    rd(FT_TRIG_STOP_DONE) & 0x1 != 0
}

/// Read the running frame count, which climbs while frames run.
pub fn frame_count() -> u16 {
    (rd(FT_FRAME_CNT) & 0xFFFF) as u16
}

/// Read the running chirp count.
pub fn chirp_count() -> u32 {
    rd(CT_CHIRP_CNT)
}

/// Read the running burst count.
pub fn burst_count() -> u16 {
    (rd(CT_BURST_CNT) & 0xFFFF) as u16
}

/// Read the RFS's own `(burst_start, frame_start, frame_free_run)` counters.
/// A `frame_start` stuck at 0 while [`frame_count`] climbs means the RFS is
/// dead, so check [`crate::fecss::rfs_fault_status`].
pub fn rfs_frame_status() -> (u32, u32, u32) {
    (
        rd(RFS_SENS_STATUS),        // w_BurstStartCount
        rd(RFS_SENS_STATUS + 0x08), // w_FrameStartCount
        rd(RFS_SENS_STATUS + 0x10), // w_FrameFreeRunCount
    )
}

const HWASS_SHRD_RAM_MEM_INIT: usize = ADCBUF_CTRL + 0x38;
const HWASS_SHRD_RAM_MEM_DONE: usize = ADCBUF_CTRL + 0x3C;
const HWASS_SHRD_RAM_MEM_STATUS: usize = ADCBUF_CTRL + 0x40;
const MEM_BIT: u32 = 1 << 0;
const MEM_INIT_TRIES: u32 = 1_000_000;

/// Mem-initialize the HWASS shared RAM behind the ADCBUF, returning `false` on
/// poll timeout. Required before capture, since until it runs the DFE's ADC
/// writes never commit. Needs [`crate::clock::enable_hwass`] first.
pub fn hwass_mem_init() -> bool {
    if !wait_bit(HWASS_SHRD_RAM_MEM_STATUS, false) {
        return false; // an earlier init is still in flight
    }
    wr(HWASS_SHRD_RAM_MEM_DONE, MEM_BIT); // clear a stale done
    if !wait_bit(HWASS_SHRD_RAM_MEM_DONE, false) {
        return false;
    }
    wr(HWASS_SHRD_RAM_MEM_INIT, MEM_BIT);
    if !wait_bit(HWASS_SHRD_RAM_MEM_DONE, true) {
        return false;
    }
    wr(HWASS_SHRD_RAM_MEM_DONE, MEM_BIT); // ack
    true
}

fn wait_bit(addr: usize, want: bool) -> bool {
    for _ in 0..MEM_INIT_TRIES {
        if (rd(addr) & MEM_BIT != 0) == want {
            return true;
        }
    }
    false
}

/// Compute the bytes the ADCBUF allots per RX per chirp, rounded up to the
/// required 16-byte alignment.
pub fn adc_bytes_per_rx(num_adc_samples: u16) -> usize {
    (num_adc_samples as usize * 2).div_ceil(16) * 16
}

/// Configure the ADCBUF for non-interleaved real int16 capture of the RX
/// channels in `rx_mask`, where RX n lands [`adc_bytes_per_rx`] apart. Call
/// before [`start`].
pub fn adcbuf_config(rx_mask: u16, num_adc_samples: u16) {
    let units = (adc_bytes_per_rx(num_adc_samples) / 16) as u32; // 16-B units
    wr(ADCBUFCFG1, ADCBUFCFG1_RESET | (((rx_mask as u32) & 0xF) << 6));
    wr(ADCBUFCFG2, units << 16); // ADDRX0 = 0, ADDRX1 = units
    wr(ADCBUFCFG3, (2 * units) | ((3 * units) << 16));
}

/// Point the ADCBUF read window at the ping (`false`) or pong (`true`) half.
/// The window is read-only for the M4: writing it raises a FEC access event
/// that asserts the RFS firmware.
pub fn adcbuf_view(pong: bool) {
    let base = rd(ADCBUFCFG1);
    wr(
        ADCBUFCFG1,
        if pong {
            base & !ADCBUFCFG1_PIPOOVRCNT
        } else {
            base | ADCBUFCFG1_PIPOOVRCNT
        },
    );
}

/// Read up to `dest.len()` int16 samples for RX region `rx` from the current
/// [`adcbuf_view`] window. Returns the number of samples read.
pub fn adcbuf_read(rx: usize, num_adc_samples: u16, dest: &mut [i16]) -> usize {
    let base = ADCBUF_RD + rx * adc_bytes_per_rx(num_adc_samples);
    let n = (num_adc_samples as usize).min(dest.len());
    for (i, d) in dest.iter_mut().take(n).enumerate() {
        *d = unsafe { core::ptr::read_volatile((base + i * 2) as *const i16) };
    }
    n
}
