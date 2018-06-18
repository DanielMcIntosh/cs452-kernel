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
    SYSCALL_AWAIT,
    SYSCALL_QUIT,
    SYSCALL_INTERRUPTS_OFF,
    SYSCALL_INTERRUPTS_ON,
    SYSCALL_DESTROY,
    SYSCALL_CREATE_ARGUMENT,
    SYSCALL_STORE_VALUE,
    SYSCALL_GET_VALUE,

    SYSCALL_INTERRUPT = 100
} Syscall;

typedef enum {
    VALUE_IDLE,
    VALUE_STACK_AVG,
    VALUE_STACK_MAX,
    NUM_VALUES
} StorableValue;

typedef struct valuestore { int values[NUM_VALUES]; } ValueStore;

// ==== K1 ====
int Create(int priority, void (*code)());
int MyTid();
int MyParentTID();
void Pass();
void Exit();

// ==== K2 ====
int Send(int tid, void *msg, int msglen, void *reply, int rplen);
int Receive(int *tid, void *msg, int msglen);
int Reply(int tid, void *reply, int rplen);

// ==== K3 ====
int AwaitEvent(int eventType);

// ==== K4 ====
int Quit();
int EnterCriticalSection();
int ExitCriticalSection();
int Destroy();
int CreateWithArgument(int priority, void (*code)(int), int argument);
int StoreValue(StorableValue tag, int value);
int GetValue(StorableValue tag);

#endif
