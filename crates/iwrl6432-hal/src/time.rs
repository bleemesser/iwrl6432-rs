use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use xwrl64xx_pac::{self as pac, AppRti};

use crate::clock;

/// Must match the `embassy-time` `tick-hz-1_000` feature (1 tick = 1 ms).
const TICKS_PER_SEC: u32 = 1000;
/// Prescaler so `RTIFRC0` ticks at `TICKS_PER_SEC`: `rate = RTI_CLK / (RTICPUC0+1)`.
const RTICPUC0: u32 = clock::RTI_CLK_HZ / TICKS_PER_SEC - 1;

const COMP0: u32 = 1 << 0; // RTISETINT / RTICLEARINT / RTIINTFLAG compare-0 bit
const CNT0EN: u32 = 1 << 0; // RTIGCTRL counter-0 enable

#[inline]
fn rti() -> &'static pac::app_rti::RegisterBlock {
    unsafe { &*AppRti::PTR }
}

struct State {
    queue: Queue,
}

struct RtiDriver {
    state: Mutex<RefCell<State>>,
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: RtiDriver = RtiDriver {
        state: Mutex::new(RefCell::new(State { queue: Queue::new() })),
    }
);

impl RtiDriver {
    /// Arm compare-0 for absolute tick `at`. Returns `false` if `at` is already
    /// past and the caller re-drains. True if armed, or if `at` is `u64::MAX`.
    fn set_alarm(&self, at: u64) -> bool {
        let r = rti();
        if at == u64::MAX {
            r.rticlearint().write(|w| unsafe { w.bits(COMP0) });
            return true;
        }
        r.rticomp0().write(|w| unsafe { w.bits(at as u32) });
        r.rtiintflag().write(|w| unsafe { w.bits(COMP0) }); // clear stale flag
        r.rtisetint().write(|w| unsafe { w.bits(COMP0) });
        self.now() < at // false if the deadline slipped past while programming
    }
}

/// Read a monotonic 40 MHz timestamp (25 ns ticks) for ISR-duration
/// diagnostics, since this M4 does not implement DWT CYCCNT. Wraps every
/// ~107 s, so it is only good for interval math.
pub fn ticks_40mhz() -> u32 {
    let r = rti();
    loop {
        let f1 = r.rtifrc0().read().bits();
        let uc = r.rtiuc0().read().bits();
        if r.rtifrc0().read().bits() == f1 {
            return f1.wrapping_mul(RTICPUC0 + 1).wrapping_add(uc);
        }
    }
}

impl Driver for RtiDriver {
    fn now(&self) -> u64 {
        rti().rtifrc0().read().bits() as u64
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut s = self.state.borrow(cs).borrow_mut();
            if s.queue.schedule_wake(at, waker) {
                let mut next = s.queue.next_expiration(self.now());
                while !self.set_alarm(next) {
                    next = s.queue.next_expiration(self.now());
                }
            }
        });
    }
}

/// Configure and start the tickless RTI time driver. Call once, after clocks
/// are up. `RTIFRC0` is 32-bit, so `now()` wraps after ~49.7 days.
pub fn init(dp: &pac::Peripherals) {
    clock::enable_rti(dp);

    let r = rti();
    r.rtigctrl().write(|w| unsafe { w.bits(0) }); // stop
    r.rtiuc0().write(|w| unsafe { w.bits(0) });
    r.rtifrc0().write(|w| unsafe { w.bits(0) });
    r.rticpuc0().write(|w| unsafe { w.bits(RTICPUC0) });
    r.rticompctrl().write(|w| unsafe { w.bits(0) }); // compare COMP0 against FRC0
    r.rtiudcp0().write(|w| unsafe { w.bits(0) }); // one-shot (no periodic auto-add)
    r.rtiintflag().write(|w| unsafe { w.bits(0xF) });
    r.rticlearint().write(|w| unsafe { w.bits(0xF) }); // no alarm armed yet

    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::MUXED_APPSS_RTI1_RTI2_INT_REQ0) };

    r.rtigctrl().modify(|rr, w| unsafe { w.bits(rr.bits() | CNT0EN) }); // start
}

/// RTI compare-0 handler (NVIC IRQ 43). Overrides the weak `device.x` alias.
#[unsafe(no_mangle)]
extern "C" fn MUXED_APPSS_RTI1_RTI2_INT_REQ0() {
    rti().rtiintflag().write(|w| unsafe { w.bits(COMP0) }); // ack (write-1-to-clear)
    critical_section::with(|cs| {
        let mut s = DRIVER.state.borrow(cs).borrow_mut();
        let mut next = s.queue.next_expiration(DRIVER.now());
        while !DRIVER.set_alarm(next) {
            next = s.queue.next_expiration(DRIVER.now());
        }
    });
}
