use cortex_m::asm;

use crate::{capture, fecss, sensor};

/// RX channels to power + capture (bit per RX).
const RX_MASK: u16 = 0x7;
/// TX channels to power (bit per TX).
const TX_MASK: u16 = 0x3;

/// Bring-up stage that failed, with the underlying error.
#[derive(Clone, Copy, Debug)]
pub enum BringupError {
    Power(fecss::PowerError),
    /// RFS M3 did not report boot Pass, carrying what it did report.
    RfsBoot(fecss::BootStatus),
    Mailbox {
        stage: &'static str,
        err: fecss::MailboxError,
    },
    Config(sensor::ConfigError),
    /// HWASS shared-RAM mem-init timed out (DFE writes would never commit).
    HwassMemInit,
    /// Frame timer never honored the start trigger.
    StartNotHonored,
}

/// Calibration + temperature results from a successful [`start`].
pub struct BringupReport {
    /// Successful when `run_status` equals [`fecss::CAL_MASK_COLDBOOT`].
    pub fact_cal: fecss::CalStatus,
    pub runtime_cal: fecss::CalStatus,
    pub temp: Option<fecss::TempMeas>,
    /// Average deg C used to pick the runtime-cal bin (25 if `temp` is `None`).
    pub temp_c: i16,
}

fn mb<T>(stage: &'static str, r: Result<T, fecss::MailboxError>) -> Result<T, BringupError> {
    r.map_err(|err| BringupError::Mailbox { stage, err })
}

/// Run the full cold bring-up. On success chirps land in the [`capture`] ring
/// and [`sensor::frame_count`] climbs. Takes ~100 ms, mostly calibration.
///
/// Caller must have run [`crate::clock::init_core`],
/// [`crate::clock::enable_hwass`] and, on flash boots,
/// [`crate::normalize_after_bootloader`].
pub fn start(cfg: &sensor::RadarConfig) -> Result<BringupReport, BringupError> {
    fecss::power_on().map_err(BringupError::Power)?;
    sensor::stop();
    sensor::reset();

    // an RFS booted over a running frame timer wedges, hence the stop and reset above
    let bs = fecss::rfs_boot();
    if bs != fecss::BootStatus::Pass {
        return Err(BringupError::RfsBoot(bs));
    }

    // config must be programmed before the RFS does per-frame analog setup
    mb("rf_pwr_onoff", fecss::rf_pwr_onoff(RX_MASK, TX_MASK, 0))?;
    sensor::configure(cfg).map_err(BringupError::Config)?;
    mb("apll_on", fecss::apll_on())?;
    let fact_cal = mb("fact_cal", fecss::fact_cal())?;

    // temperatures only read valid once RF analog and the APLL are up
    fecss::temp_config(0x311); // RX, TX, PM and DIG sensors
    let temp = fecss::temp_trig().ok();
    let temp_c = temp
        .as_ref()
        .map(|t| ((t.temp_c[0] as i32 + t.temp_c[4] as i32 + t.temp_c[8] as i32) / 3) as i16)
        .unwrap_or(25);
    let runtime_cal = mb("runtime_cal", fecss::runtime_cal(fecss::temp_bin_for(temp_c)))?;

    if !sensor::hwass_mem_init() {
        return Err(BringupError::HwassMemInit);
    }
    sensor::adcbuf_config(RX_MASK, cfg.common.num_adc_samples);
    capture::init(
        cfg.common.num_adc_samples,
        RX_MASK.count_ones() as usize,
        cfg.capture_format,
        cfg.frame.chirps_per_burst,
        cfg.frame.bursts_per_frame,
    );

    sensor::start(sensor::FrameTrigMode::SwImmediate);
    // the frame timer latches the start key within a frame period
    for _ in 0..200 {
        if sensor::start_honored() {
            return Ok(BringupReport { fact_cal, runtime_cal, temp, temp_c });
        }
        asm::delay(crate::clock::CORE_CLK_HZ / 1_000); // ~1 ms
    }
    Err(BringupError::StartNotHonored)
}

/// Stop frames at the next frame boundary.
pub fn stop() {
    sensor::stop();
}

/// Retune a radar that [`start`] already brought up. Calibration is not re-run,
/// since it is per-band rather than per-profile.
pub fn reconfigure(cfg: &sensor::RadarConfig) -> Result<(), BringupError> {
    sensor::stop();
    // the stop key latches at the next frame boundary, so wait out a
    // worst-case frame period (profiles stay under 50 ms) before resetting
    asm::delay(crate::clock::CORE_CLK_HZ / 1_000 * 80);
    sensor::reset();
    sensor::configure(cfg).map_err(BringupError::Config)?;
    sensor::adcbuf_config(RX_MASK, cfg.common.num_adc_samples);
    capture::init(
        cfg.common.num_adc_samples,
        RX_MASK.count_ones() as usize,
        cfg.capture_format,
        cfg.frame.chirps_per_burst,
        cfg.frame.bursts_per_frame,
    );
    sensor::start(sensor::FrameTrigMode::SwImmediate);
    for _ in 0..200 {
        if sensor::start_honored() {
            return Ok(());
        }
        asm::delay(crate::clock::CORE_CLK_HZ / 1_000);
    }
    Err(BringupError::StartNotHonored)
}
