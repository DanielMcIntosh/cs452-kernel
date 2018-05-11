#ifndef CIRCLEBUFFER_H
#define CIRCLEBUFFER_H

#define CIRCLEBUFSIZE 400

struct circlebuffer {
    char buf[CIRCLEBUFSIZE];
    int rd; // rd is the index of the next space to read
    int wr; // wr is the index of the next space to write.
    int empty;
};

void cb_init(struct circlebuffer *cb);
int cb_full(struct circlebuffer *cb);
int cb_empty(struct circlebuffer *cb);
int cb_read(struct circlebuffer *cb, char *c);
int cb_read_number(struct circlebuffer *cb, int *i); // NOTE: reads 1 past the number rn

int cb_write(struct circlebuffer *cb, char c);
int cb_backspace(struct circlebuffer *cb);
int cb_write_string(struct circlebuffer *cb, char *s);
int cb_write_number(struct circlebuffer *cb, unsigned int num, unsigned int base);


#endif
