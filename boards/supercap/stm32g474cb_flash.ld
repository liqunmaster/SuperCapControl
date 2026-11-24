ENTRY(Reset_Handler)
/* xB 器件含 128 KiB 闪存，SRAM1 与 SRAM2 合计 96 KiB。CCM 为独立存储区，DMA 缓冲区应放在 SRAM。 */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
    CCM (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
    .ARM.extab (READONLY) : { *(.ARM.extab*) } > FLASH
    .ARM.exidx (READONLY) : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
    .preinit_array (READONLY) : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
    .init_array (READONLY) : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
    .fini_array (READONLY) : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
    _sidata = LOADADDR(.data);
    .data : { . = ALIGN(4); _sdata = .; *(.data*) *(.RamFunc*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
    .heap_stack (NOLOAD) : { . = ALIGN(8); end = .; _end = .; . += 0x800; . += 0x1000; . = ALIGN(8); } > RAM
    ASSERT(_estack >= ADDR(.heap_stack) + SIZEOF(.heap_stack), "RAM overflow")
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
