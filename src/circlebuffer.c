#include <ts7200.h>
#include <circlebuffer.h>
#include <err.h>

void cb_init(struct circlebuffer *cb, char *buf, int size) {
    cb->buf = buf;
    cb->size = size;
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
    cb->rd = (cb->rd+1) % cb->size;
    if (cb->rd == cb->wr) 
        cb->empty = 1;

    return 0;
}

int cb_peek(struct circlebuffer *cb, char *c){
    if (cb_empty(cb)) {
        return 1;
    }

    *c = cb->buf[cb->rd];
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

int cb_read_match(circlebuffer_t *cb, char *str) {
    char c;
    while (!cb_empty(cb) && *str != '\0') {
        cb_read(cb, &c);
        if (c != *str++) {
            return ERR_TEXT_MISMATCH;
        }
    }
    if (*str != '\0') {
        return ERR_TEXT_MISMATCH;
    }
    return 0;
}

int cb_write(struct circlebuffer *cb, char c){
    if (cb_full(cb)) {
        return 1;
    }

    cb->buf[cb->wr] = c;
    cb->wr = (cb->wr + 1) % cb->size;
    cb->empty = 0;
    return 0;
}

int cb_backspace(struct circlebuffer *cb){
    if (cb_empty(cb)){
        return 1;
    }

    cb->wr = (cb->wr - 1) % cb->size;
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

//d = base^(num_digits_to_print). This function will pad with 0's if necessary.
int cb_write_fixed_size_number(struct circlebuffer *cb, unsigned int num, unsigned int base, unsigned int d) {
    int dgt;
    while (d != 0) {
        dgt = num / d; // get digit
        num %= d;
        d /= base;
        int err = cb_write(cb, dgt + ( dgt < 10 ? '0' : ('A' - 10)));
        if (err) {
            return err;
        }
    }
    return 1;
}

int cb_write_number(struct circlebuffer *cb, int num, unsigned int base) {
    if (num < 0) {
        cb_write(cb, '-');
        num *= -1;
    }
    unsigned int d = 1;

    while( (num / d) >= base) {
        d  *= base; // find max digit
    }

    return cb_write_fixed_size_number(cb, num, base, d);
}
