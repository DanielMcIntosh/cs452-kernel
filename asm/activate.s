    .globl   activate
    .p2align 2
    .type    activate,%function
activate:
    .fnstart
    @ save kernel state
    stmdb sp!, {r4-r12,lr} @ Not Saved: PC, SP
    MRS r4, CPSR
    str r4, [sp, #-4]!
    str r0, [sp, #-4]! @ r0 contains a pointer to the task struct

    ldr r4, [r0, #24] @ CSPR_USR
    MSR SPSR, r4 @ Set SPSR_svc to CPSR_user, which will return it to user mode once movs is called.
    ldr lr, [r0, #16] @LR

    MSR CPSR_c,#0xDF @ System mode
        ldr sp, [r0, #20] @ Stack Pointer 
        ldmia sp!, {r0-r12,lr} @ Reload registers from User Stack
    MSR CPSR_c, #0xD3 @ Supervisor Mode

    movs pc, lr

    mov pc, #0 @ should never get here
    .globl IRQ_ENTRY_POINT
IRQ_ENTRY_POINT:
    mov sp, #1 @ leave something in IRQ sp

    .globl KERNEL_ENTRY_POINT
KERNEL_ENTRY_POINT:
    MSR CPSR_c, #0xDF @ System mode
        stmdb sp!, {r0-r12,lr}@ Push user registers onto active task stack
        mov r4, sp  @Save Stack Pointer
    MSR CPSR_c, #0xD3 @ Supervisor Mode - even if we were in IRQ mode, we return to svc mode.

    ldmia sp!, {r5} @load the task we're coming out of's TD*

    str r4, [r5, #20] @sp

    
@ Retrieve syscall ID
    MSR CPSR_c, #0xD2 @ IRQ mode
        mov r0, sp
        mov sp, #0
        sub r1, lr, #4
        MRS r2, SPSR
    MSR CPSR_c, #0xD3 @ SVC mode

    teq r0, #1
    beq irq
    @ If this isn't IRQ, we get the value from here
    MRS r4, SPSR
    str r4, [r5, #24] @CPSR_usr
    ldr r0, [lr, #-4]
    and r0, r0, #0xFFFFFF @ Mask SWI bits from Return Value
    str lr, [r5, #16] @ Link Register from SWI
    b after
irq:
    @ If this is an IRQ, we use the IRQ return value
    mov r0, #100
    str r2, [r5, #24] @CPSR_usr
    str r1, [r5, #16] @ Link Register from IRQ
after:

    ldmia sp!, {r4}
    MSR CPSR, r4
    ldmia sp!, {r4-r12,lr}
    mov pc, lr
