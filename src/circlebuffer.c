#include <ts7200.h>
#include <circlebuffer.h>
#include <err.h>
#include <debug.h>
#include <syscall.h>

void cb_init(struct circlebuffer * restrict cb, char *buf, int size) {
    cb->buf = buf;
    cb->size = size;
    cb->rd = 0;
    cb->wr = 0;
    cb->empty = 1;
}

inline int cb_full(struct circlebuffer *cb){
    return (cb->rd == cb->wr) && !cb->empty;
}

inline int cb_empty(struct circlebuffer *cb){
    return (cb->rd == cb->wr) && cb->empty;
}

inline void cb_flush(struct circlebuffer *cb) {
    cb->wr = cb->rd;
    cb->empty = 1;
}

inline int cb_avail_for_read(struct circlebuffer *cb) {
    return cb_full(cb) ? cb->size : (cb->wr - cb->rd + cb->size) % cb->size;
}

inline int cb_avail_for_write(struct circlebuffer *cb) {
    return cb_empty(cb) ? cb->size : (cb->rd - cb->wr + cb->size) % cb->size;
}

int cb_read(struct circlebuffer * restrict cb, char * restrict c){
    if (cb_empty(cb)) {
        ASSERT(FALSE, "empty cb during read");
        return 1;
    }

    *c = cb->buf[cb->rd];
    cb->rd = (cb->rd+1) % cb->size;
    if (cb->rd == cb->wr) 
        cb->empty = 1;

    return 0;
}

int cb_peek(struct circlebuffer * restrict cb, char * restrict c){
    if (cb_empty(cb)) {
        return 1;
    }

    *c = cb->buf[cb->rd];
    return 0;
}

int __attribute__((const)) cb_ch2d(char ch){
    return ch - '0';
}

// NOTE: cb_read_number reads 1 past the number;
int cb_read_number(struct circlebuffer * restrict cb, int * restrict i){
    int num, digit;

    int r = 1;

    num = 0;
    char c;
    while(cb_peek(cb, &c) == 0) {
        ASSERT(cb_read(cb, &c) == 0, "read failed from peeked cb");
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

int cb_read_match(circlebuffer_t * restrict cb, const char * restrict str) {
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

int cb_read_int(struct circlebuffer *cb, int *i){ // reads 4-char int
    *i = 0;
    char k = 0;
    for (int y = 0; y < 4; y++) {
        int err = cb_read(cb, &k);
        if (err) return err;
        *i |= (((int) k) << (y * 8));
    }
    return 0;
}

int cb_write(struct circlebuffer *cb, char c){
    if (cb_full(cb)) {
        ASSERT(FALSE, "writing to full cb");
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

    cb->wr = (cb->wr + cb->size - 1) % cb->size;
    cb->empty = (cb->wr == cb->rd);
    return 0;
}

int cb_write_string(struct circlebuffer * restrict cb, const char * restrict s) {
    while (*s != '\0' && cb_write(cb, *s) == 0) {
        s++;
    }

    return (*s != '\0');
}

int cb_write_int(struct circlebuffer *cb, int i) {
    for (int y = 0; y < 4; y++) {
        int err = cb_write(cb, ((unsigned int)i) & 0xFF);
        if (err) return err;
        i >>= 8;
    }
    return 0;
}

//d = base^(num_digits_to_print). This function will pad with 0's if necessary.
inline int cb_write_fixed_size_number(struct circlebuffer *cb, unsigned int num, unsigned int base, unsigned int d) {
    unsigned char dgt;
    int f = 0;
    while (d != 0) {
        dgt = (num / d) & 0xF; // get digit
        num %= d;
        d /= base;
        int err = cb_write(cb, dgt + ( dgt < 10 ? '0' : ('A' - 10)));
        f++;
        if (err) {
            return err;
        }
    }
    return f;
}

int cb_write_number(struct circlebuffer *cb, int num, unsigned int base) {
    int f = 0;
    if (num < 0) {
        cb_write(cb, '-');
        num *= -1;
        f++;
    }
    unsigned int d = 1, u_num = (unsigned int)num;
    
    while( (u_num / d) >= base) {
        d  *= base; // find max digit
    }

    return f + cb_write_fixed_size_number(cb, u_num, base, d);
}
