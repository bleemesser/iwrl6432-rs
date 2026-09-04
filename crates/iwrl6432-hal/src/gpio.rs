use core::convert::Infallible;

use crate::{clock, pac, pinmux};

#[inline]
fn base() -> usize {
    pac::TopGio::PTR as usize
}

// ports A..H have identical register blocks at a fixed stride, addressed as a
// grid rather than through the PAC's flat accessors
const GIOGCR: usize = 0x00; // bit 0 = module out of reset
const PORT0_OFF: usize = 0x34;
const PORT_STRIDE: usize = 0x20;
const DIR: usize = 0x00; // 1 = output
const DIN: usize = 0x04; // live pin level (read-only)
const DOUT: usize = 0x08; // driven level (read/write)
const SET: usize = 0x0C; // write-1-to-set
const CLR: usize = 0x10; // write-1-to-clear

/// A GIO port (`A` = GPIO bits 0..=7, `B` the next 8, ...).
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    A = 0, B, C, D, E, F, G, H,
}

/// Logic level of a pin.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

/// Input pull configuration.
#[derive(Clone, Copy)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// A pin's GIO location (`port`/`bit`) plus the `pad` that routes to it and the
/// pin-mux `mode` that selects GPIO on that pad.
#[derive(Clone, Copy)]
pub struct PinDef {
    pub port: Port,
    pub bit: u8,
    pub pad: pinmux::Pad,
    pub mode: u32,
}

/// The IWRL6432BOOST USER_LED / USER_SW pin, active-high. Reaches the SoC only
/// with board switch S1.6 off, since on routes the pad to the J2 SPI header.
pub const GPIO5: PinDef = PinDef {
    port: Port::A,
    bit: 5,
    pad: pinmux::Pad::Av,
    mode: 0,
};

#[inline]
fn port_reg(port: Port, field: usize) -> *mut u32 {
    (base() + PORT0_OFF + (port as usize) * PORT_STRIDE + field) as *mut u32
}

#[inline]
fn gio_out_of_reset() {
    let gcr = (base() + GIOGCR) as *mut u32;
    unsafe { core::ptr::write_volatile(gcr, core::ptr::read_volatile(gcr) | 1) };
}

/// A GPIO configured as a push-pull output.
pub struct Output {
    set: *mut u32,
    clr: *mut u32,
    dout: *mut u32,
    mask: u32,
}

impl Output {
    /// Configure `pin` as an output driving `level`. Un-gates the GIO clock, which
/// must happen before anything touches the register block.
    pub fn new(dp: &pac::Peripherals, pin: PinDef, level: Level) -> Self {
        clock::enable_gpio(dp);
        pinmux::configure(dp, pin.pad, pinmux::pad_cfg(pin.mode));

        let mask = 1u32 << pin.bit;
        let out = Output {
            set: port_reg(pin.port, SET),
            clr: port_reg(pin.port, CLR),
            dout: port_reg(pin.port, DOUT),
            mask,
        };
        gio_out_of_reset();
        let dir = port_reg(pin.port, DIR);
        unsafe { core::ptr::write_volatile(dir, core::ptr::read_volatile(dir) | mask) };
        match level {
            Level::Low => out.set_low(),
            Level::High => out.set_high(),
        }
        out
    }

    #[inline]
    pub fn set_high(&self) {
        unsafe { core::ptr::write_volatile(self.set, self.mask) };
    }

    #[inline]
    pub fn set_low(&self) {
        unsafe { core::ptr::write_volatile(self.clr, self.mask) };
    }

    #[inline]
    pub fn toggle(&self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    /// The currently driven level (from `GIODOUT`, not the live pin).
    #[inline]
    pub fn is_set_high(&self) -> bool {
        unsafe { core::ptr::read_volatile(self.dout) & self.mask != 0 }
    }
}

/// A GPIO configured as an input.
pub struct Input {
    din: *const u32,
    mask: u32,
}

impl Input {
    /// Configure `pin` as an input with the given internal `pull`.
    pub fn new(dp: &pac::Peripherals, pin: PinDef, pull: Pull) -> Self {
        clock::enable_gpio(dp);
        // force the input buffer on via IE_OVERRIDE_CTRL and IE_OVERRIDE
        const IE_FORCE: u32 = (1 << 4) | (1 << 5);
        let pull_bits = match pull {
            Pull::None => pinmux::PIN_PULL_DISABLE,
            Pull::Up => 1 << 9,
            Pull::Down => 0,
        };
        pinmux::configure(dp, pin.pad, pin.mode | IE_FORCE | pull_bits);

        let mask = 1u32 << pin.bit;
        gio_out_of_reset();
        let dir = port_reg(pin.port, DIR);
        unsafe { core::ptr::write_volatile(dir, core::ptr::read_volatile(dir) & !mask) };
        Input {
            din: port_reg(pin.port, DIN) as *const u32,
            mask,
        }
    }

    /// The live pin level (from `GIODIN`).
    #[inline]
    pub fn is_high(&self) -> bool {
        unsafe { core::ptr::read_volatile(self.din) & self.mask != 0 }
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(Input::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(Input::is_low(self))
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_high(&mut self) -> Result<(), Infallible> {
        Output::set_high(self);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Infallible> {
        Output::set_low(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, Infallible> {
        Ok(Output::is_set_high(self))
    }
    fn is_set_low(&mut self) -> Result<bool, Infallible> {
        Ok(!Output::is_set_high(self))
    }
}
