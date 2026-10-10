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
    /// RFS M3 didn't report boot Pass; carries what it did report.
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
    /// Success = `run_status == `[`fecss::CAL_MASK_COLDBOOT`], `res_status`
    /// carrying the validity bits (0xEF on this board).
    pub fact_cal: fecss::CalStatus,
    pub runtime_cal: fecss::CalStatus,
    pub temp: Option<fecss::TempMeas>,
    /// Average deg C used to pick the runtime-cal bin (25 if `temp` is `None`).
    pub temp_c: i16,
}

fn mb<T>(stage: &'static str, r: Result<T, fecss::MailboxError>) -> Result<T, BringupError> {
    r.map_err(|err| BringupError::Mailbox { stage, err })
}

/// Full cold-to-streaming bring-up, in the SDK's cold-boot order: FECSS power
/// on -> frame timer stop+reset (an RFS booted over a running frame timer
/// wedges) -> RFS boot -> RF power -> chirp/frame config -> APLL -> factory +
/// runtime cal -> ADCBUF + [`capture::init`] -> frame start.
///
/// On success chirps land in the [`capture`] ring and [`sensor::frame_count`]
/// climbs. Caller must have run [`crate::clock::init_core`],
/// [`crate::clock::enable_hwass`] and (on flash boots)
/// [`crate::normalize_after_bootloader`]. Takes ~100 ms, mostly calibration.
pub fn start(cfg: &sensor::RadarConfig) -> Result<BringupReport, BringupError> {
    fecss::power_on().map_err(BringupError::Power)?;
    sensor::stop();
    sensor::reset();

    let bs = fecss::rfs_boot();
    if bs != fecss::BootStatus::Pass {
        return Err(BringupError::RfsBoot(bs));
    }

    // Config must be programmed before the RFS does per-frame analog setup.
    mb("rf_pwr_onoff", fecss::rf_pwr_onoff(RX_MASK, TX_MASK, 0))?;
    sensor::configure(cfg).map_err(BringupError::Config)?;
    mb("apll_on", fecss::apll_on())?;
    let fact_cal = mb("fact_cal", fecss::fact_cal())?;

    // Temperatures are valid now (RF analog + APLL up); pick the cal bin from
    // the RX0/TX0/PM average, falling back to a room-temperature bin.
    fecss::temp_config(0x311); // RX + TX + PM + DIG sensors
    let temp = fecss::temp_trig().ok();
    let temp_c = temp
        .as_ref()
        .map(|t| ((t.temp_c[0] as i32 + t.temp_c[4] as i32 + t.temp_c[8] as i32) / 3) as i16)
        .unwrap_or(25);
    let runtime_cal = mb(
        "runtime_cal",
        fecss::runtime_cal(fecss::temp_bin_for(temp_c)),
    )?;

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
    // The FT latches the start key within a frame period.
    for _ in 0..200 {
        if sensor::start_honored() {
            return Ok(BringupReport {
                fact_cal,
                runtime_cal,
                temp,
                temp_c,
            });
        }
        asm::delay(crate::clock::CORE_CLK_HZ / 1_000); // ~1 ms
    }
    Err(BringupError::StartNotHonored)
}

/// Stop frames at the next frame boundary.
pub fn stop() {
    sensor::stop();
}

/// Reconfigure a radar that [`start`] already brought up (RFS booted, RF
/// powered, calibrated): stop + reset the frame timer, push the new
/// chirp/frame config over the mailbox, re-init the ADCBUF + capture ring,
/// restart frames. Calibration is not re-run (the cal is per-band, not
/// per-profile).
pub fn reconfigure(cfg: &sensor::RadarConfig) -> Result<(), BringupError> {
    sensor::stop();
    // The stop key latches at the next frame boundary; wait out a worst-case
    // frame period (profiles stay <= 50 ms) before resetting the FT.
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
