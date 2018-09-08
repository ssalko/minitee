ENTRY(vectors_start);

MEMORY
{
	ram (rwx) : ORIGIN = 0x0e000000, LENGTH = 0x1000000
}

SECTIONS
{
    .text :
    {
        __text_start = .;
        __isr_vector_start = .;
        *(.isr_vector)
        __isr_vector_end = .;
        *(.text*)
        *(.rodata*)
        __text_end = .;
    } > ram
    .data : /*AT(ADDR(.text) + SIZEOF(.text))*/
    {
        __data_start = .;
        *(vtable)
        *(.data*)
        __data_end = .;
    } > ram
    .bss :
    {
        __bss_start = .;
        *(.bss*) 
		__bss_end = .;
		__end = .;
    } > ram
}

PROVIDE(__stack = __end);

PROVIDE(__HEAP_START = __end + 0x10000 );
