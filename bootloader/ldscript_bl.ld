/*
       linker file for AM32 bootloader
*/

/* Entry Point */
ENTRY(ResetISR)

_estack = 0x20001000;    /* end of RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
	PROGRAM_FLASH	(rx)   	: ORIGIN = 0x0, LENGTH = 0x4000 - 32	/* 16K bytes for bootloader */
	DEVINFO 		(r) 	: ORIGIN = ORIGIN(PROGRAM_FLASH) + LENGTH(PROGRAM_FLASH), LENGTH = 32 
	EEPROM			(rx)	: ORIGIN = 0x20000 - 0x2000, LENGTH = 0x2000
  	SRAM 			(rwx)	: ORIGIN = 0x20000000, LENGTH = 0x6000 /* 24K bytes (alias RAM) */  
  	SRAMX0 			(rwx) 	: ORIGIN = 0x4000000, LENGTH = 0x2000 /* 8K bytes (alias RAM2) */  
  	SRAMX1 			(rwx) 	: ORIGIN = 0x4002000, LENGTH = 0x1000 /* 4K bytes (alias RAM3) */  
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >PROGRAM_FLASH

  .devinfo :
  {
    . = ALIGN(4);
        KEEP (*(.devinfo))
    . = ALIGN(4);
  } >DEVINFO

     /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > PROGRAM_FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >PROGRAM_FLASH

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > PROGRAM_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > PROGRAM_FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >PROGRAM_FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >PROGRAM_FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >PROGRAM_FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);
  
      /* DATA section for SRAMX0 */

	.data_RAM2 : ALIGN(4)
	{
	    FILL(0xff)
	    PROVIDE(__start_data_RAM2 = .) ;
	    PROVIDE(__start_data_SRAMX0 = .) ;
	    *(.ramfunc.$RAM2)
	    *(.ramfunc.$SRAMX0)
	    *(.data.$RAM2)
	    *(.data.$SRAMX0)
	    *(.data.$RAM2.*)
	    *(.data.$SRAMX0.*)
	    . = ALIGN(4) ;
	    PROVIDE(__end_data_RAM2 = .) ;
	    PROVIDE(__end_data_SRAMX0 = .) ;
	 } > SRAMX0 AT>PROGRAM_FLASH
	
	/* DATA section for SRAMX1 */
	
	.data_RAM3 : ALIGN(4)
	{
	    FILL(0xff)
	    PROVIDE(__start_data_RAM3 = .) ;
	    PROVIDE(__start_data_SRAMX1 = .) ;
	    *(.ramfunc.$RAM3)
	    *(.ramfunc.$SRAMX1)
	    *(.data.$RAM3)
	    *(.data.$SRAMX1)
	    *(.data.$RAM3.*)
	    *(.data.$SRAMX1.*)
	    . = ALIGN(4) ;
	    PROVIDE(__end_data_RAM3 = .) ;
	    PROVIDE(__end_data_SRAMX1 = .) ;
	 } > SRAMX1 AT>PROGRAM_FLASH

    /* Main DATA section (SRAM) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_SRAM = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_SRAM = .) ;
    } > SRAM AT>PROGRAM_FLASH

    /* BSS section for SRAMX0 */
    .bss_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_SRAMX0 = .) ;
       *(.bss.$RAM2)
       *(.bss.$SRAMX0)
       *(.bss.$RAM2.*)
       *(.bss.$SRAMX0.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_SRAMX0 = .) ;
    } > SRAMX0 AT> SRAMX0

    /* BSS section for SRAMX1 */
    .bss_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_SRAMX1 = .) ;
       *(.bss.$RAM3)
       *(.bss.$SRAMX1)
       *(.bss.$RAM3.*)
       *(.bss.$SRAMX1.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_SRAMX1 = .) ;
    } > SRAMX1 AT> SRAMX1
    
    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_SRAM = .) ;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_SRAM = .) ;
        PROVIDE(end = .);
    } > SRAM AT> SRAM

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x600;
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > SRAM

     _StackSize = 0x600;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > SRAM
    /* Locate actual Stack in memory map */
    .stack ORIGIN(SRAM) + LENGTH(SRAM) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > SRAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
