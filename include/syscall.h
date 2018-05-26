#ifndef SYSCALL_H
#define SYSCALL_H

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

int Send(int tid, void *msg, int msglen, void *reply, int rplen);
int Receive(int *tid, void *msg, int msglen);
int Reply(int tid, void *reply, int rplen);

#endif
