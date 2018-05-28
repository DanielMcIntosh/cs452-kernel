#include <kernel.h>
#include <bwio.h>
#include <syscall.h>

// FIXME: Optimize number of arguments for smaller syscalls
inline static __attribute__((always_inline)) int syscall_5(const int n, const int arg1, const int arg2, const int arg3, const int arg4, const int arg5){
    int ret;
// Save r0-r3 (on the stack?)
// Insert any arguments in r0-r3 (i.e. pointer for create)
__asm__(
    ASM_STACK_PUSH("lr")
    ASM_STACK_PUSH("r0")
    ASM_STACK_PUSH("r1")
    ASM_STACK_PUSH("r2")
    ASM_STACK_PUSH("r3")
    "mov r0, %[arg5]\n\t"
    ASM_STACK_PUSH("r0")
    "mov r0, %[arg1]\n\t"
    "mov r1, %[arg2]\n\t"
    "mov r2, %[arg3]\n\t"
    "mov r3, %[arg4]\n\t"
    "swi %[n]\n\t"
        :
        : [n] "i" (n), [arg1] "ri" (arg1), [arg2] "ri" (arg2), [arg3] "ri" (arg3), [arg4] "ri" (arg4), [arg5] "ri" (arg5)
        : "r0", "r1", "r2", "r3", "lr");
// Store r0 (return value)
__asm__(
    "mov %[ret], r0\n\t"
    ASM_STACK_POP("r3")
    ASM_STACK_POP("r2")
    ASM_STACK_POP("r1")
    ASM_STACK_POP("r0")
    ASM_STACK_POP("lr")
    : [ret] "=r"(ret)
    :
    : "r0", "r1", "r2", "r3", "lr");
// Re-load r0-r3
    return ret;
}

inline static __attribute__((always_inline)) int syscall_2(const int n, const int arg1, const int arg2){
    return syscall_5(n, arg1, arg2, 0, 0, 0);
}
inline static __attribute__((always_inline)) int syscall_3(const int n, const int arg1, const int arg2, const int arg3){
    return syscall_5(n, arg1, arg2, arg3, 0, 0);
}

void Pass(){
    syscall_2(SYSCALL_PASS, 0, 0);
}

void Exit(){
    syscall_2(SYSCALL_EXIT, 0, 0);
}

int MyTid(){
    return syscall_2(SYSCALL_TID, 0, 0);
}

int MyParentTID(){
    return syscall_2(SYSCALL_PTID, 0, 0);
}

int Create(int priority, void (*code)()){
    return syscall_2(SYSCALL_CREATE, priority, (int)code);
}

int Send(int tid, void *msg, int msglen, void *reply, int rplen){
    return syscall_5(SYSCALL_SEND, tid, (int) msg, msglen, (int) reply, rplen);
}

int Receive(int *tid, void *msg, int msglen){
    return syscall_3(SYSCALL_RECEIVE, (int) tid, (int) msg, msglen);
}

int Reply(int tid, void *reply, int rplen){
    return syscall_3(SYSCALL_REPLY, tid, (int) reply, rplen);
}

