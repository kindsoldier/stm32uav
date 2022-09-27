
MEMORY {
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

EXTERN (vector_table)

ENTRY(reset_handler)

SECTIONS
{
        .text : {
                *(.vectors)
                *(.text*)
                . = ALIGN(4);
                *(.rodata*)
                . = ALIGN(4);
        } >FLASH

        .preinit_array : {
                . = ALIGN(4);
                __preinit_array_start = .;
                KEEP (*(.preinit_array))
                __preinit_array_end = .;
        } >FLASH

        .init_array : {
                . = ALIGN(4);
                __init_array_start = .;
                KEEP (*(SORT(.init_array.*)))
                KEEP (*(.init_array))
                __init_array_end = .;
        } >FLASH

        .fini_array : {
                . = ALIGN(4);
                __fini_array_start = .;
                KEEP (*(.fini_array))
                KEEP (*(SORT(.fini_array.*)))
                __fini_array_end = .;
        } >FLASH

        . = ALIGN(4);
        _etext = .;

        .data : {
                _data = .;
                *(.data*)
                . = ALIGN(4);
                _edata = .;
        } >SRAM AT >FLASH

        _data_loadaddr = LOADADDR(.data);

        .bss : {
                *(.bss*)
                *(COMMON)
                . = ALIGN(4);
                _ebss = .;
        } >SRAM


        . = ALIGN(4);
        _end = .;
}

PROVIDE(_stack = ORIGIN(SRAM) + LENGTH(SRAM));
