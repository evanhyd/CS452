SECTIONS {
	. = 0x80000;            /* start text at this location */
	.text.boot : {          /* boot code must come first */
		KEEP(*(.text.boot))
	}
	.text : {               /* rest of code */
		KEEP(*(.text*))
	}
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
		__init_array_end = .;
	}

	. = 0x200000;           /* start data at this location for MMU setup */
	.rodata : {
		KEEP(*(.rodata*))
	}
	.data : {
		KEEP(*(.data*))
	}
	.bss : {
		KEEP(*(.bss*))
	}
}
