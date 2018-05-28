    .globl   activate
    .p2align 2
    .type    activate,%function
activate:
    .fnstart
    @ save kernel state
    stmdb sp!, {r4-r12,lr} @ Not Saved: PC, SP
    MRS r4, CPSR
    stmdb sp!, {r4}
    stmdb sp!, {r0}
    @ r0 contains a pointer to the task struct
    ldr r4, [r0, #24] @ CSPR_USR
    MSR SPSR_cxsf, r4 @ Set SPSR_svc to CPSR_user, which will return it to user mode once movs is called.
       
    ldr lr, [r0, #16] @LR

    MSR CPSR_c,#0xDF @ System mode
    ldr sp, [r0, #20] @ Stack Pointer 
    ldmia sp!, {r4-r12,lr} @ Reload registers from User Stack
    MSR CPSR_c, #0xD3 @ Supervisor Mode

    ldr r0, [r0, #28] @ Return Value

    movs pc, lr
    mov pc, #0
    .globl KERNEL_ENTRY_POINT
KERNEL_ENTRY_POINT:
    str r0, [sp, #-4]! @we'll need to borrow r0 for the argument
    MSR CPSR_c, #0xDF @ System mode
    ldr r0, [sp], #4 @pop arg 5 from stack
    stmdb sp!, {r4-r12,lr}@ Push user registers onto active task stack
    mov r4, sp  @Save Stack Pointer
    MSR CPSR_c, #0xD3 @ Supervisor Mode

    ldr r6, [sp], #4 @Get back original r0
    ldmia sp!, {r5} @load the task we're coming out of's TD*

    str r4, [r5, #20]

    str lr, [r5, #16] @ Link Register from SWI

    MRS r4, SPSR
    str r4, [r5, #24]
    
    str r6, [r5, #32] @r6 is original r0
    str r1, [r5, #36] @ Put args into task struct
    str r2, [r5, #40]
    str r3, [r5, #44]
    str r0, [r5, #48]


    ldr r0, [lr, #-4]
    and r0, r0, #0xFFFFFF @ Mask SWI bits from Return Value

    ldmia sp!, {r4}
    MSR CPSR_cxsf, r4
    ldmia sp!, {r4-r12,lr}
    mov pc, lr
