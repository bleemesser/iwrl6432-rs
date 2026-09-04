# xwrl64xx-pac

Peripheral access crate for the TI xWRL64xx (IWRL6432), owned by this
workspace. Forked from [abeanater/xwrl64xx-pac](https://github.com/abeanater/xwrl64xx-pac)
(MIT, see LICENSE), which generated it from TI's targetdb XML via tixml2svd +
svd2rust.

Local changes on top of the upstream generation:
- Real interrupt vector table (`Interrupt` enum, `__INTERRUPTS[64]`,
  `device.x`), derived from SDK `cslr_intr.h`; enabled by the `rt` feature.
- Generation inputs (SVD XML, sanitize script, single-file lib.rs) removed;
  regenerate from the upstream repo if ever needed.

The `repository` and `homepage` fields point at this workspace because that is
where the fork lives. Upstream is `abeanater/xwrl64xx-pac`.
