#include <kernel.h>

#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4]\n\t"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4\n\t"

inline static __attribute__((always_inline)) int syscall(const int n, const int arg1, const int arg2){
    int ret;
// Save r0-r3 (on the stack?)
// Insert any arguments in r0-r3 (i.e. pointer for create)
__asm__(
    ASM_STACK_PUSH("r0")
    ASM_STACK_PUSH("r1")
    ASM_STACK_PUSH("r2")
    ASM_STACK_PUSH("r3")
    "mov r0, %[arg1]\n\t"
    "mov r1, %[arg2]\n\t"
    "swi %[n]\n\t"
        :
        : [n] "i" (n), [arg1] "ri" (arg1), [arg2] "ri" (arg2));
// Store r0 in memory (return value? what's r0 here)
__asm__(
    "mov %[ret], r1\n\t"
    ASM_STACK_POP("r3")
    ASM_STACK_POP("r2")
    ASM_STACK_POP("r1")
    ASM_STACK_POP("r0")
    : [ret] "=r"(ret));
// Re-load r0-r3
    return ret;
}

void Pass(){
    syscall(SYSCALL_PASS, 0, 0);
}

void Exit(){
    syscall(SYSCALL_EXIT, 0, 0);
}

int MyTid(){
    return syscall(SYSCALL_TID, 0, 0);
}

int MyParentTID(){
    return syscall(SYSCALL_PTID, 0, 0);
}

int Create(int priority, void (*code)()){
    return syscall(SYSCALL_CREATE, priority, (int)code);
}
