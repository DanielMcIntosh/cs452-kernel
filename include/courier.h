#ifndef _COURIER_H_
#define _COURIER_H_

typedef void (*CourierSendFn)(void *);

void task_courier_rp_null(int other_tid, int size);
void task_courier_rp_rpmsg(int other_tid, int size);

void __attribute__((noreturn)) forever_w_courier(int *tid, void *in_msg, int in_msg_size, int other_tid, int cb_size, int out_msg_size, int rp_size, void (*body)(CourierSendFn));

#endif //_COURIER_H_
