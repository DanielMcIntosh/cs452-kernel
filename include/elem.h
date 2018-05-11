#ifndef ELEM_H
#define ELEM_H

typedef struct elem {
    void *item;
    unsigned int size;
} elem_t;

#define ELEM(x) {x, sizeof(*x)}
#define E(X) x, sizeof(*x)

#endif
