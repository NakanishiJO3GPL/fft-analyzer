MEMORY {
    /* Main flash */
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 2M

    /* AXI SRAM hosts stack/.data/.bss so RTT is debugger-visible */
    AXISRAM(rwx) : ORIGIN = 0x24000000, LENGTH = 512K
    RAM(rwx)     : ORIGIN = 0x24000000, LENGTH = 512K

    /* DTCM (unused unless explicitly placed) */
    DTCM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K

    /* Extra SRAM banks */
    SRAM1  (rwx) : ORIGIN = 0x30000000, LENGTH = 128K
    SRAM2  (rwx) : ORIGIN = 0x30020000, LENGTH = 128K
    SRAM3  (rwx) : ORIGIN = 0x30040000, LENGTH = 32K
    SRAM4  (rwx) : ORIGIN = 0x38000000, LENGTH = 16K

    /* Backup SRAM */
    BSRAM  (rwx) : ORIGIN = 0x38800000, LENGTH = 4K

    /* Instruction TCM */
    ITCM   (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
}

_stack_start = ORIGIN(AXISRAM) + LENGTH(AXISRAM);

SECTIONS {
    .axisram (NOLOAD) : ALIGN(8) {
        *(.axisram .axisram.*);
        . = ALIGN(8);
    } > AXISRAM

    .sram1 (NOLOAD) : ALIGN(4) {
        *(.sram1 .sram1.*);
        . = ALIGN(4);
    } > SRAM1

    .sram2 (NOLOAD) : ALIGN(4) {
        *(.sram2 .sram2.*);
        . = ALIGN(4);
    } > SRAM2

    .sram3 (NOLOAD) : ALIGN(4) {
        *(.sram3 .sram3.*);
        . = ALIGN(4);
    } > SRAM3

    .sram4 (NOLOAD) : ALIGN(4) {
        *(.sram4 .sram4.*);
        . = ALIGN(4);
    } > SRAM4

    /* Optional: reserve DTCM-backed buffers here if needed
    .dtcm (NOLOAD) : ALIGN(4) {
        *(.dtcm .dtcm.*);
        . = ALIGN(4);
    } > DTCM
    */
};
