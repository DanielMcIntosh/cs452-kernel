#ifndef KERNEL_H
#define KERNEL_H

int Create(int priority, void (*code)());
int MyTid();
int MyParentTID();
void Pass();
void Exit();

#endif
