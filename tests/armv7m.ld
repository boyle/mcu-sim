MEMORY
{
  rom      (rx)  : ORIGIN = 0x00000000, LENGTH = 8K
  ram      (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__top_rom = ORIGIN(rom) + LENGTH(rom);
__top_ram = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
    .text : ALIGN(4)
    {
        FILL(0xff)
        KEEP(*(.vectors .vectors.*))
        *(.text.*)
        *(.rodata .rodata.*)
    } > rom

    . = ALIGN(4);
    _etext = .;

    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
    } > ram

    .data : ALIGN(4)
    {
        FILL(0xff)
        _sdata = .;
        *(.ramfunc .ramfunc.*);
        *(vtable);
        *(.data*);
        . = ALIGN(4);
        _edata = .;
    } > ram AT > rom

    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        _sbss = . ;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = . ;
        PROVIDE(_end = .);
    } > ram

    PROVIDE(_stack_top = __top_ram - 0);
}
