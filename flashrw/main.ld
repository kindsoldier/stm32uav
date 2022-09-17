
MEMORY {
    FLASH0 (rx) : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH1 (rw) : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH3 (rx) : ORIGIN = 0x08008000, LENGTH = 480K
    SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
}

PROVIDE(_stack = ORIGIN(SRAM) + LENGTH(SRAM));
PROVIDE(_config = ORIGIN(FLASH1));

stack_size = 40k;
heap_size = 40k;

_stack_start = ORIGIN(SRAM) + LENGTH(SRAM);
_stack_end = _stack_start - stack_size;

EXTERN (vector_table)

ENTRY(reset_handler)

SECTIONS {
        text : {
                *(.vectors)
                . = ALIGN(4);
                *(.rodata*)
                . = ALIGN(4);
        } >FLASH0

        .text : {
                *(.text*)
                . = ALIGN(4);
        } >FLASH3

        .preinit_array : {
                . = ALIGN(4);
                __preinit_array_start = .;
                KEEP (*(.preinit_array))
                __preinit_array_end = .;
        } >FLASH0

        .init_array : {
                . = ALIGN(4);
                __init_array_start = .;
                KEEP (*(SORT(.init_array.*)))
                KEEP (*(.init_array))
                __init_array_end = .;
        } >FLASH0

        .fini_array : {
                . = ALIGN(4);
                __fini_array_start = .;
                KEEP (*(.fini_array))
                KEEP (*(SORT(.fini_array.*)))
                __fini_array_end = .;
        } >FLASH0

        . = ALIGN(4);
        _etext = .;

        .data : {
                _data = .;
                . = ALIGN(4);
                *(.data*)
                . = ALIGN(4);
                _edata = .;
        } >SRAM AT >FLASH0

        _data_loadaddr = LOADADDR(.data);

        .bss : {
                *(.bss*)
                *(COMMON)
                . = ALIGN(4);
                _ebss = .;
        } >SRAM

        . = ALIGN(4);

        .heap : {
           _heap_start = .;
           . = . + heap_size;
           _heap_end = .;
        } > SRAM

        . = ALIGN(4);
        _end = .;
}
