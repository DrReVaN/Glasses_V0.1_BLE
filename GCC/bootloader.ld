ENTRY(Reset_Handler)
MEMORY {
 FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0xe000
 RAM (rwx) : ORIGIN = 0x20000008, LENGTH = 0x77F8
 RETAIN (rw) : ORIGIN = 0x20007800, LENGTH = 0x80
 SHARED (rw) : ORIGIN = 0x20030000, LENGTH = 0x2800
}
PHDRS {
 flash PT_LOAD FLAGS(5);
 ramcode PT_LOAD FLAGS(5);
 ram PT_LOAD FLAGS(6);
 bss PT_LOAD FLAGS(6);
 shared PT_LOAD FLAGS(6);
 retain PT_LOAD FLAGS(6);
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
 .isr_vector : { KEEP(*(.isr_vector)) } > FLASH :flash
 .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH :flash
 .ARM.extab : { *(.ARM.extab*) } > FLASH :flash
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH :flash
 .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH :flash
 .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH :flash
 .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH :flash
 _siramcode = LOADADDR(.ramcode);
 .ramcode : { . = ALIGN(4); _sramcode = .; *(.RamFunc*) . = ALIGN(4); _eramcode = .; } > RAM AT> FLASH :ramcode
 _sishared = LOADADDR(.shared);
 _sidata = LOADADDR(.data);
 .data : { . = ALIGN(4); _sdata = .; *(.data*) *(SYSTEM_DRIVER_CONTEXT) . = ALIGN(4); _edata = .; } > RAM AT> FLASH :ram
 /* BSS has its own RAM-only segment; input alignment must not change .data's flash load mapping. */
 .bss (NOLOAD) : AT(ADDR(.bss)) { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) *(BLE_APP_CONTEXT) *(BLE_DRIVER_CONTEXT) *(TIMERSERVER_CONTEXT) . = ALIGN(4); _ebss = .; } > RAM :bss
 .noinit (NOLOAD) : { *(.noinit*) } > RETAIN :retain
 .shared : { . = ALIGN(4); _sshared = .; KEEP(*(MAPPING_TABLE)) *(MB_MEM1) *(MB_MEM2) . = ALIGN(4); _eshared = .; } > SHARED AT> FLASH :shared
 .heap_stack (NOLOAD) : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . += 512; PROVIDE(_heap_limit = .); . += 2048; } > RAM :NONE
 /DISCARD/ : { *(.note.GNU-stack) }
}
ASSERT(SIZEOF(.isr_vector) == 0x13C, "WB35 vector table must contain 79 entries")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH), "Image overlaps reserved flash")
