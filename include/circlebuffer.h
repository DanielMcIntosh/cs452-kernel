#ifndef CIRCLEBUFFER_H
#define CIRCLEBUFFER_H

typedef struct circlebuffer {
    char *buf;
    int size;
    int rd; // rd is the index of the next space to read
    int wr; // wr is the index of the next space to write.
    int empty;
} circlebuffer_t;

void cb_init(struct circlebuffer *cb, char * buf, int size);
int cb_full(struct circlebuffer *cb);
int cb_empty(struct circlebuffer *cb);
void cb_flush(struct circlebuffer *cb);
int cb_avail_for_read(struct circlebuffer *cb);
int cb_avail_for_write(struct circlebuffer *cb);

int cb_read(struct circlebuffer *cb, char *c);
int cb_peek(struct circlebuffer *cb, char *c);
int cb_read_number(struct circlebuffer *cb, int *i); // NOTE: reads 1 past the number rn
int cb_read_match(circlebuffer_t *cb, char *str);
int cb_read_int(struct circlebuffer *cb, int *i); // reads 4-char int
int cb_read_struct(struct circlebuffer *cb, void *s, int size);

int cb_write(struct circlebuffer *cb, char c);
int cb_backspace(struct circlebuffer *cb);
int cb_write_string(struct circlebuffer *cb, char *s);
int cb_write_struct(struct circlebuffer *cb, void *s, int size);
int cb_write_int(struct circlebuffer *cb, int i);

//d = base^(num_digits_to_print). This function will pad with 0's if necessary.
int cb_write_fixed_size_number(struct circlebuffer *cb, unsigned int num, unsigned int base, unsigned int d);
int cb_write_number(struct circlebuffer *cb, int num, unsigned int base);


#endif
