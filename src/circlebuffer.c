#include <ts7200.h>
#include <circlebuffer.h>

void cb_init(struct circlebuffer *cb) {
    cb->rd = 0;
    cb->wr = 0;
    cb->empty = 1;
}

int cb_full(struct circlebuffer *cb){
    return (cb->rd == cb->wr) && !cb->empty;
}

int cb_empty(struct circlebuffer *cb){
    return (cb->rd == cb->wr) && cb->empty;
}

int cb_read(struct circlebuffer *cb, char *c){
    if (cb_empty(cb)) {
        return 1;
    }

    *c = cb->buf[cb->rd];
    cb->rd = (cb->rd+1) % CIRCLEBUFSIZE;
    if (cb->rd == cb->wr) 
        cb->empty = 1;

    return 0;
}

int cb_ch2d(char ch){
    return ch - '0';
}

// NOTE: cb_read_number reads 1 past the number; TODO: fix that?
int cb_read_number(struct circlebuffer *cb, int *i){
    int num, digit;

    int r = 1;

    num = 0;
    char c;
    while(cb_read(cb, &c) == 0) {
        digit = cb_ch2d(c);
        if (digit >= 10 || digit < 0){
            break;
        }
        r = 0;
        num = num * 10 + digit;
    }

    *i = num;
    return r;

}

int cb_write(struct circlebuffer *cb, char c){
    if (cb_full(cb)) {
        return 1;
    }

    cb->buf[cb->wr] = c;
    cb->wr = (cb->wr + 1) % CIRCLEBUFSIZE;
    cb->empty = 0;
    return 0;
}

int cb_backspace(struct circlebuffer *cb){
    if (cb_empty(cb)){
        return 1;
    }

    cb->wr = (cb->wr - 1) % CIRCLEBUFSIZE;
    cb->empty = (cb->wr == cb->rd);
    return 0;
}

int cb_write_string(struct circlebuffer *cb, char * s) {
    while (*s) {
        cb_write(cb, *s);
        s++;
    }

    return 0;
}

int cb_write_number(struct circlebuffer *cb, unsigned int num, unsigned int base) {
    int n = 0;
    int dgt;
    unsigned int d = 1;

    while( (num / d) >= base) d  *= base; // find max digit
    while (d != 0) {
        dgt = num / d; // get digit
        num %= d;
        d /= base;
        if (n || dgt > 0 || d == 0) {
            int err = cb_write(cb, dgt + ( dgt < 10 ? '0' : 'A' - 10));
            if (err) {
                return err;
            }
            ++n;
        }
    }
    return 1;
}
