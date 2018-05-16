    .globl   activate
    .p2align 2
    .type    activate,%function
activate:
    .fnstart
    @ save kernel state
    stmdb sp!, {r4-r12,lr} @ Not Saved: PC, SP
    MRS r4, CPSR
    stmdb sp!, {r4}

    @ r0 contains a pointer to the task struct

    mov r5, r0

    mov r0, #1
    mov r1, r5
    bl bwputr

    ldr r4, [r5, #28] @ CSPR_USR
    mov r0, #1
    mov r1, r4
    bl bwputr
    MSR SPSR_cxsf, r4 @ 5. Set SPSR_svc to CPSR_user, which will return it to user mode once movs is called.
       
    MRS R0,CPSR
    BIC R0,R0,#0x1F
    ORR R0,R0,#0x1F
    MSR CPSR_c,R0

    ldr r4, [r5, #24]
    mov sp, r4 @ Reload Stack Pointer 
    @"ldmia sp, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14}\n\t" // 10. Reload registers from User Stack
    MSR CPSR_c, #0xD3 @ Supervisor Mode
    ldr r4, [r5, #20] @LR

    mov r0, #1
    mov r1, r4
    bl bwputr

    mov lr, r4
    @mov r0, %[task_ret]
    movs pc, lr

    @TODO assert
    .globl KERNEL_ENTRY_POINT
KERNEL_ENTRY_POINT:

@ KERNEL ENTER: ("after the context switch")
@ TODO Acquire Args
@ Acquire PC of active task
    @mov %[lr], lr
    @MSR CPSR_c, #0x1F @ System Mode
    @"stmdb sp!, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14}@ Push user registers onto active task stack
    @mov %[sp], sp @Save Stack Pointer
    @MSR CPSR_c, #0x13 @ Supervisor Mode
    @MRS %[cpsr_usr], SPSR
    @ldr r0, [lr, #-4]
    ldmia sp!, {r4}
    MSR CPSR_cxsf, r4
    ldmia sp!, {r4-r12,lr}
    mov pc, lr
