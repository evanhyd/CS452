.global preSyscallEntry
.global preIrqEntry
.global switchTask
.global vectors

.extern syscallEntry
.extern irqEntry
.extern placeholderEntry
.extern _start

.macro PREP_KERNEL_ENTRY
    msr SPSel, #0

    stp x0, x1, [sp, #-16]!
    stp x2, x3, [sp, #-16]!
    stp x4, x5, [sp, #-16]!
    stp x6, x7, [sp, #-16]!
    stp x8, x9, [sp, #-16]!
    stp x10, x11, [sp, #-16]!
    stp x12, x13, [sp, #-16]!
    stp x14, x15, [sp, #-16]!
    stp x16, x17, [sp, #-16]!
    stp x18, x19, [sp, #-16]!
    stp x20, x21, [sp, #-16]!
    stp x22, x23, [sp, #-16]!
    stp x24, x25, [sp, #-16]!
    stp x26, x27, [sp, #-16]!
    stp x28, x29, [sp, #-16]!

    mrs x0, ESR_EL1
    stp x30, x0, [sp, #-16]!

    mrs x1, ELR_EL1
    mrs x2, SPSR_EL1
    stp x1, x2, [sp, #-16]!

    mov x0, sp
    msr SPSel, #1

    // Important: reset kernel stack back to top (kernel doesn't maintain stack state)
    // Otherwise, will gradually consume all kernel stack space
    ldr x1, =_start
    mov sp, x1
.endm

preSyscallEntry:
    PREP_KERNEL_ENTRY

    b syscallEntry

preIrqEntry:
    PREP_KERNEL_ENTRY

    b irqEntry

switchTask:
    // Set user SP to task's SP, and switch to user mode.
    msr SP_EL0, x0
    msr SPSel, #0

    // Read the task context.
    ldp x0, x1, [sp], #16
    msr ELR_EL1, x0
    msr SPSR_EL1, x1
    ldp x30, xzr, [sp], #16
    ldp x28, x29, [sp], #16
    ldp x26, x27, [sp], #16
    ldp x24, x25, [sp], #16
    ldp x22, x23, [sp], #16
    ldp x20, x21, [sp], #16
    ldp x18, x19, [sp], #16
    ldp x16, x17, [sp], #16
    ldp x14, x15, [sp], #16
    ldp x12, x13, [sp], #16
    ldp x10, x11, [sp], #16
    ldp x8, x9, [sp], #16
    ldp x6, x7, [sp], #16
    ldp x4, x5, [sp], #16
    ldp x2, x3, [sp], #16
    ldp x0, x1, [sp], #16

    eret

.align 11
vectors:
    // Group 1 (Current EL SP0)
    .align 7
    mov x0, #1
    mov x1, #0
    b placeholderEntry
    .align 7
    mov x0, #1
    mov x1, #1
    b placeholderEntry
    .align 7
    mov x0, #1
    mov x1, #2
    b placeholderEntry
    .align 7
    mov x0, #1
    mov x1, #3
    b placeholderEntry

    // Group 2 (Current EL SPx)
    .align 7
    mov x0, #2
    mov x1, #0
    b placeholderEntry
    .align 7
    mov x0, #2
    mov x1, #1
    b placeholderEntry
    .align 7
    mov x0, #2
    mov x1, #2
    b placeholderEntry
    .align 7
    mov x0, #2
    mov x1, #3
    b placeholderEntry

    // Group 3 (Lower EL 64)
    .align 7
    b preSyscallEntry
    .align 7
    b preIrqEntry
    .align 7
    mov x0, #3
    mov x1, #2
    b placeholderEntry
    .align 7
    mov x0, #3
    mov x1, #3
    b placeholderEntry

    // Group 4 (Lower EL 32)
    .align 7
    mov x0, #4
    mov x1, #0
    b placeholderEntry
    .align 7
    mov x0, #4
    mov x1, #1
    b placeholderEntry
    .align 7
    mov x0, #4
    mov x1, #2
    b placeholderEntry
    .align 7
    mov x0, #4
    mov x1, #3
    b placeholderEntry
