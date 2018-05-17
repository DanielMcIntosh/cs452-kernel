#ifndef KERNEL_H
#define KERNEL_H

typedef enum {
	SYSCALL_CREATE,
	SYSCALL_TID,
	SYSCALL_PTID,
	SYSCALL_PASS,
	SYSCALL_EXIT,
	SYSCALL_SEND,
	SYSCALL_RECEIVE,
	SYSCALL_REPLY,
} Syscall;

int Create(int priority, void (*code)());
int MyTid();
int MyParentTID();
void Pass();
void Exit();
int Send(int tid, char *msg, int msglen, char *reply, int rplen);
int Receive(int *tid, char *msg, int msglen);
int Reply(int tid, char *reply, int rplen);

// FIXME move
#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4]!\n\t"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4\n\t"

#endif
