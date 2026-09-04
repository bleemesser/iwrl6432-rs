/* IWRL6432 APPSS Cortex-M4F memory map (from the TI SDK example linker.cmd).
 * The M4F runs entirely from on-chip RAM aliased into the code region at
 * 0x00400000; the reset vector table sits at the start of that region.
 *
 *   FLASH (RAM12) : 0x00400000, 0x58000  vectors + .text + .rodata + .data init
 *   M4F_RBL       : 0x00458000, 0x08000  reserved for ROM bootloader (left alone)
 *   RAM   (RAM3)  : 0x00460000, 0x20000  .bss/.data/heap/stack
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00058000
    RAM   (rwx) : ORIGIN = 0x00460000, LENGTH = 0x00020000
}

/* Stack grows down from the top of RAM3. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
