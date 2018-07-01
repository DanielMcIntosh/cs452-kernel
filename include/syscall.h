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
    SYSCALL_CREATE_2_ARGS,
    SYSCALL_STORE_VALUE,
    SYSCALL_GET_VALUE,

    SYSCALL_INTERRUPT = 100
} Syscall;

typedef enum {
    PRIORITY_INIT,
    PRIORITY_NOTIFIER,
    PRIORITY_WAREHOUSE,
    PRIORITY_HIGHEST,
    PRIORITY_HIGHER = PRIORITY_HIGHEST+2,
    PRIORITY_HIGH = PRIORITY_HIGHER+2,
    PRIORITY_MID = PRIORITY_HIGH+2,
    PRIORITY_LOW = PRIORITY_MID+2,
    PRIORITY_LOWER = PRIORITY_LOW+2,
    PRIORITY_LOWEST = 14,
    PRIORITY_IDLE,
    NUM_PRIORITIES
} Priority;

typedef enum {
    VALUE_IDLE,
    VALUE_STACK_AVG,
    VALUE_STACK_MAX,
    NUM_VALUES
} StorableValue;

typedef struct valuestore { int values[NUM_VALUES]; } ValueStore;

// ==== K1 ====
int Create(Priority priority, void (*code)());
int MyTid();
int MyParentTID();
void Pass();
void Exit();

// ==== K2 ====
int Send(int tid, const void *msg, int msglen, void *reply, int rplen);
int Receive(int *tid, void *msg, int msglen);
int Reply(int tid, const void *reply, int rplen);

// ==== K3 ====
int AwaitEvent(int eventType);

// ==== K4 ====
int Quit();
int EnterCriticalSection();
int ExitCriticalSection();
int Destroy();

int CreateWithArgument(Priority priority, void (*code)(int), int argument);
int CreateWith2Args(Priority priority, void (*code)(int, int), int arg0, int arg1);

int StoreValue(StorableValue tag, int value);
int GetValue(StorableValue tag);

#endif
