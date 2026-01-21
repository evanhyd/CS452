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


	. = 0x2FFFDFF0;  /* 256 mega bytes of stack frame */
	__task_stack_begin = .;
	. = 0x3FFFDFF0;
	__task_stack_end = .;

	. = 0x3FFFDFF0;  /* 8192 bytes of task descriptors */
	__task_descriptors_begin = .;
	. = 0x3FFFFFF0;
	__task_descriptors_end = .;
}
