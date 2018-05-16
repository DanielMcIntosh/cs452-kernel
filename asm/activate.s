    .globl   activate
    .p2align 2
    .type    activate,%function
activate:
    .fnstart
@	mov	ip, sp
@	stmfd	sp!, {fp, ip, lr, pc}
@	sub	fp, ip, #4
@	sub	sp, sp, #8
@	str	r0, [fp, #-24]

    @ save kernel state
    stmdb sp!, {r4-r12,lr} @ Not Saved: PC, SP
    MRS r4, CPSR
    stmdb sp!, {r4}
    stmdb sp!, {r0}
    @ r0 contains a pointer to the task struct
    mov r5, r0
    ldr r4, [r5, #28] @ CSPR_USR
    MSR SPSR_cxsf, r4 @ 5. Set SPSR_svc to CPSR_user, which will return it to user mode once movs is called.
       
    ldr r4, [r5, #20] @LR
    mov lr, r4

    ldr r4, [r5, #32] @ Return Value
    @ If return values are buggy, consider moving to inside system mode
    mov r0, r4


    MSR CPSR_c,#0xDF
    ldr r4, [r5, #24]
    mov sp, r4 @ Stack Pointer 
    ldmia sp!, {r4-r12,lr} @ 10. Reload registers from User Stack
    MSR CPSR_c, #0xD3 @ Supervisor Mode

    movs pc, lr
    mov pc, #0
    .globl KERNEL_ENTRY_POINT
KERNEL_ENTRY_POINT:

@ KERNEL ENTER: ("after the context switch")
@ TODO Acquire Args
    MSR CPSR_c, #0xDF
    stmdb sp!, {r4-r12,lr}@ Push user registers onto active task stack
    mov r4, sp  @Save Stack Pointer
    MSR CPSR_c, #0xD3 @ Supervisor Mode
    ldmia sp!, {r5} @load the task we're coming out of's TD*

    str r4, [r5, #24]

    mov r4, lr @ Link Register from SWI
    str r4, [r5, #20]

    MRS r4, SPSR
    str r4, [r5, #28]

    mov r3, r0
    ldr r0, [lr, #-4]
    and r0, r0, #0xFFFFFF @ Mask SWI bits from Return Value

    ldmia sp!, {r4}
    MSR CPSR_cxsf, r4
    ldmia sp!, {r4-r12,lr}
    mov pc, lr
