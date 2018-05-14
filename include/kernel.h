#ifndef KERNEL_H
#define KERNEL_H

#define SYSCALL_CREATE 0x1
#define SYSCALL_TID 0x2
#define SYSCALL_PTID 0x3
#define SYSCALL_PASS 0x4
#define SYSCALL_EXIT 0x5

int Create(int priority, void (*code)());
int MyTid();
int MyParentTID();
void Pass();
void Exit();

// FIXME move
#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4]!\n\t"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4\n\t"

#endif
