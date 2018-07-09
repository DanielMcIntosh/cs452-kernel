#include <courier.h>
#include <message.h>
#include <syscall.h>
#include <debug.h>
#include <util.h>
#include <circlebuffer.h>

typedef enum courier_state{
    COURIER_BUSY,
    COURIER_READY,
    SEND_PENDING,
} CourierState;

void task_courier_rp_null(int other_tid, int size) {
    int parent_tid = MyParentTID();
    char buf[size];
    FOREVER {
        Send(parent_tid, NULL, 0, buf, size);
        Send(other_tid, buf, size, NULL, 0);
    }
}

void task_courier_rp_rpmsg(int other_tid, int size) {
    int parent_tid = MyParentTID();
    char buf[size];
    ReplyMessage rp = {0, 0};
    FOREVER {
        Send(parent_tid, &rp, sizeof(rp), buf, size);
        Send(other_tid, buf, size, &rp, sizeof(rp));
    }
}

void task_courier(int other_tid, int sizes) {
    unsigned int out_msg_size = ((unsigned int)sizes >> 16) & 0xFFFF;
    unsigned int in_msg_size = ((unsigned int)sizes >> 8) & 0xFF;
    unsigned int rp_size = (unsigned int)sizes & 0xFF;

    int parent_tid = MyParentTID();
    char buf[out_msg_size];
    char dummy[in_msg_size];
    char dummy2[rp_size];
    FOREVER {
        Send(parent_tid, dummy, in_msg_size, buf, out_msg_size);
        Send(other_tid, buf, out_msg_size, dummy2, rp_size);
    }
}

void forever_w_courier(int * restrict tid, void * restrict in_msg, unsigned int in_msg_size, int other_tid, int cb_size, unsigned int out_msg_size, unsigned int rp_size, void (*body)(CourierSendFn)) {
    circlebuffer_t cb_courier;
    char cb_courier_buf[cb_size];
    cb_init(&cb_courier, cb_courier_buf, sizeof(cb_courier_buf));
    CourierState courier_state = COURIER_BUSY;

    int courier_tid = CreateWith2Args(PRIORITY_WAREHOUSE, &task_courier, other_tid, (out_msg_size << 16) | (in_msg_size << 8) | rp_size);

    //READ: https://gcc.gnu.org/onlinedocs/gcc/Nested-Functions.html
    void send_to_courier(void *outgoing) {
        if (courier_state == COURIER_READY) {
            Reply(courier_tid, outgoing, out_msg_size);
            courier_state = COURIER_BUSY;
        } else {
            cb_write_struct(&cb_courier, outgoing, out_msg_size);
            courier_state = SEND_PENDING;
        }
    }

    FOREVER {
        Receive(tid, in_msg, in_msg_size);
        if (*tid == courier_tid) {
            if (courier_state == SEND_PENDING) {
                char buf[out_msg_size];
                cb_read_struct(&cb_courier, buf, out_msg_size);
                Reply(courier_tid, buf, out_msg_size);
                courier_state = cb_empty(&cb_courier) ? COURIER_BUSY : SEND_PENDING;
            } else {
                courier_state = COURIER_READY;
            }
        }
        else {
            body(&send_to_courier);
        }
    }
}
