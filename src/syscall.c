#include <kernel.h>
#include <bwio.h>
#include <syscall.h>
#include <debug.h>

// FIXME: Optimize number of arguments for smaller syscalls
inline static __attribute__((always_inline)) int syscall_5(const int n, const int arg1, const int arg2, const int arg3, const int arg4, const int arg5){
    int ret;
    LOGF("Expected Arguments: %d, %d, %d, %d, %d\r\n", arg1, arg2, arg3, arg4, arg5);
__asm__(
    "stmdb sp!, {r0-r3,lr}\n\t"
    "mov r0, %[arg5]\n\t"
    ASM_STACK_PUSH("r0")
    "mov r0, %[arg1]\n\t"
    "mov r1, %[arg2]\n\t"
    "mov r2, %[arg3]\n\t"
    "mov r3, %[arg4]\n\t"
    "swi %[n]\n\t"
        :
        : [n] "i" (n), [arg1] "ri" (arg1), [arg2] "ri" (arg2), [arg3] "ri" (arg3), [arg4] "ri" (arg4), [arg5] "ri" (arg5)
        : "r0", "r1", "r2", "r3", "lr", "sp");
// Store r0 (return value)
__asm__ volatile ( // FIXME: why does this need to be volatile? why was gcc dropping it? bc it's reloading lr and sp from fp later anyway?
    "mov %[ret], r0\n\t"
    "add sp, sp, #4\n\t" // FIXME: pop arg5 in activate?
    "ldmia sp!, {r0-r3,lr}\n\t"
    : [ret] "=r"(ret)
    :
    : "r0", "r1", "r2", "r3", "lr", "sp");
// Re-load r0-r3
    return ret;
}

inline static __attribute__((always_inline)) int syscall_0(const int n){
    return syscall_5(n, 0, 0, 0, 0, 0);
}
inline static __attribute__((always_inline)) int syscall_2(const int n, const int arg1, const int arg2){
    return syscall_5(n, arg1, arg2, 0, 0, 0);
}
inline static __attribute__((always_inline)) int syscall_3(const int n, const int arg1, const int arg2, const int arg3){
    return syscall_5(n, arg1, arg2, arg3, 0, 0);
}
inline static __attribute__((always_inline)) int syscall_4(const int n, const int arg1, const int arg2, const int arg3, const int arg4){
    return syscall_5(n, arg1, arg2, arg3, arg4, 0);
}

void Pass(){
    syscall_0(SYSCALL_PASS);
}

void Exit(){
    syscall_0(SYSCALL_EXIT);
}

int MyTid(){
    return syscall_0(SYSCALL_TID);
}

int MyParentTID(){
    return syscall_0(SYSCALL_PTID);
}

int Create(Priority priority, void (*code)()){
    return syscall_2(SYSCALL_CREATE, priority, (int)code);
}

int Send(int tid, void *msg, int msglen, void *reply, int rplen){
    return syscall_5(SYSCALL_SEND, tid, (int) msg, msglen, (int) reply, rplen);
}

int Receive(int * restrict tid, void * restrict msg, int msglen){
    return syscall_3(SYSCALL_RECEIVE, (int) tid, (int) msg, msglen);
}

int Reply(int tid, void *reply, int rplen){
    return syscall_3(SYSCALL_REPLY, tid, (int) reply, rplen);
}

int AwaitEvent(int eventType){
    return syscall_2(SYSCALL_AWAIT, eventType, 0);
}

int Quit(){
    return syscall_0(SYSCALL_QUIT);
}

int EnterCriticalSection(){
    return syscall_0(SYSCALL_INTERRUPTS_OFF);
}

int ExitCriticalSection(){
    return syscall_0(SYSCALL_INTERRUPTS_ON);
}

int Destroy(){
    return syscall_0(SYSCALL_DESTROY);
}

int CreateWithArgument(Priority priority, void (*code)(int), int argument){
    return syscall_3(SYSCALL_CREATE_ARGUMENT, priority, (int) code, argument);
}

int CreateWith2Args(Priority priority, void (*code)(int, int), int arg0, int arg1){
    return syscall_4(SYSCALL_CREATE_2_ARGS, priority, (int) code, arg0, arg1);
}

int StoreValue(StorableValue tag, int value){
    return syscall_2(SYSCALL_STORE_VALUE, tag, value);
}

int GetValue(StorableValue tag){
    return syscall_2(SYSCALL_GET_VALUE, tag, 0);
}
