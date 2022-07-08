/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                1024Kbytes FLASH and 560Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
** All rights reserved.</center></h2>
**
** This software component is licensed by ST under BSD 3-Clause license,
** the "License"; You may not use this file except in compliance with the
** License. You may obtain a copy of the License at:
**                        opensource.org/licenses/BSD-3-Clause
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
_isr_start = ORIGIN(FLASH_FIRMWARE);
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x1000 ;      /* required amount of heap  */
_Min_Stack_Size = 0x1000 ; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  /* FLASH_BOOT (rx)        : ORIGIN = 0x08000000,   LENGTH = 128K */
  FLASH_APP (r)             : ORIGIN = 0x08020000,   LENGTH = 128K
  FLASH_CHECKSUM (r)        : ORIGIN = 0x08040000,   LENGTH = 0x0080 
  FLASH_GUIDS (r)           : ORIGIN = 0x08040080,   LENGTH = 0x0080
  FLASH_FIRMWARE (rx)       : ORIGIN = 0x08040100,   LENGTH = 256K - 0x0100
  FLASH_CACHE (rx)          : ORIGIN = 0x080A0000,   LENGTH = 256K
  ITCMRAM (xrw)             : ORIGIN = 0x00000000,   LENGTH = 64K
  DTCMRAM (xrw)             : ORIGIN = 0x20000000,   LENGTH = 128K
  RAM_D1  (xrw)             : ORIGIN = 0x24000000,   LENGTH = 320K
  RAM_D2  (xrw)             : ORIGIN = 0x30000000,   LENGTH = 32K
  RAM_D3  (xrw)             : ORIGIN = 0x38000000,   LENGTH = 16K
}

/* Define output sections */
SECTIONS
{
  /* GUIDs */
  .guids :
  {
    . = ALIGN(4);
    KEEP(*(.guids))
    . = ALIGN(4);
  } >FLASH_GUIDS

  /* The vector table goes first into FLASH_FIRMWARE */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Interrupt vectors */
    . = ALIGN(4);
  } >FLASH_FIRMWARE

  /* The program code and other data goes into FLASH_FIRMWARE */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH_FIRMWARE

  /* Constant data goes into FLASH_FIRMWARE */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH_FIRMWARE

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH_FIRMWARE
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH_FIRMWARE

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH_FIRMWARE

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH_FIRMWARE

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH_FIRMWARE

  .pmcu :
  {
    . = ALIGN(8);
    _spmcu = .;
    KEEP(*(.pmcu))
    . = ALIGN(8);
    _epmcu = .;
  } >FLASH_FIRMWARE =0xFF

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >DTCMRAM AT> FLASH_FIRMWARE

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >DTCMRAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  .mdmalink (NOLOAD) : /* Space before ':' is critical */
  {
    . = ALIGN(8);
    _smdmalink = .;
    *(.mdmalink)
    . = ALIGN(8);
    _emdmalink = .;
  } >RAM_D1
  .ramD1 (NOLOAD) : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD1 = .;
    *(.ramD1)
    . = ALIGN(4);
    _eramD1 = .;
  } >RAM_D1
  _siramD1init = LOADADDR(.ramD1init);
  .ramD1init : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD1init = .;
    *(.ramD1init)
    . = ALIGN(4);
    _eramD1init = .;
  } >RAM_D1 AT> FLASH_FIRMWARE
  .ramD1zero : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD1zero = .;
    *(.ramD1zero)
    . = ALIGN(4);
    _eramD1zero = .;
  } >RAM_D1

.lwip_sec (NOLOAD) : {
    . = ABSOLUTE(0x30000000);
    *(.RxDecripSection) 
    . = ABSOLUTE(0x30000060);
    *(.TxDecripSection)
    . = ABSOLUTE(0x30000200);
    *(.RxArraySection) 
  } >RAM_D2 AT> FLASH_FIRMWARE 
  
.ramD2 (NOLOAD) : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD2 = .;
    *(.ramD2)
    . = ALIGN(4);
    _eramD2 = .;
  } >RAM_D2
  _siramD2init = LOADADDR(.ramD2init);
  .ramD2init : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD2init = .;
    *(.ramD2init)
    . = ALIGN(4);
    _eramD2init = .;
  } >RAM_D2 AT> FLASH_FIRMWARE
  .ramD2zero : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD2zero = .;
    *(.ramD2zero)
    . = ALIGN(4);
    _eramD2zero = .;
  } >RAM_D2

  .ramD3 (NOLOAD) : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD3 = .;
    *(.ramD3)
    . = ALIGN(4);
    _eramD3 = .;
  } >RAM_D3
  _siramD3init = LOADADDR(.ramD3init);
  .ramD3init : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD3init = .;
    *(.ramD3init)
    . = ALIGN(4);
    _eramD3init = .;
  } >RAM_D3 AT> FLASH_FIRMWARE
  .ramD3zero : /* Space before ':' is critical */
  {
    . = ALIGN(4);
    _sramD3zero = .;
    *(.ramD3zero)
    . = ALIGN(4);
    _eramD3zero = .;
  } >RAM_D3

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
