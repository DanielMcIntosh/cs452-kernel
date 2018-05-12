#include <kernel.h>

#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4];"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4;"

inline static __attribute__((always_inline)) int syscall(const int n){
    int ret;
// Save r0-r3 (on the stack?)
// Insert any arguments in r0-r3 (i.e. pointer for create)
__asm__(
    ASM_STACK_PUSH("r0")\
    ASM_STACK_PUSH("r1")\
    ASM_STACK_PUSH("r2")\
    ASM_STACK_PUSH("r3")\
    "swi %0;"
        :
        : "i" (n));
// Store r0 in memory (return value? what's r0 here)
__asm__(
    "mov %0, r1;"\
    ASM_STACK_POP("r3")\
    ASM_STACK_POP("r2")\
    ASM_STACK_POP("r1")\
    ASM_STACK_POP("r0")\
    :"=r"(ret));
// Re-load r0-r3
    return ret;
}

void Pass(){
    syscall(SYSCALL_PASS);
}

void Exit(){
    syscall(SYSCALL_EXIT);
}

int MyTid(){
    return syscall(SYSCALL_TID);
}

int MyParentTID(){
    return syscall(SYSCALL_PTID);
}

int Create(int priority, void (*code)()){
    return syscall(SYSCALL_CREATE);
}
