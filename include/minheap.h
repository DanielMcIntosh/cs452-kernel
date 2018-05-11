#ifndef MINHEAP_H
#define MINHEAP_H

#include <elem.h>

typedef struct entry {
    elem_t element;
    unsigned int value;
} entry_t;

typedef struct minheap {
    entry_t* entries;
    unsigned int count;
    unsigned int size;
} minheap_t;

void mh_init(minheap_t *mh, entry_t * entries, unsigned int size);
int mh_add(minheap_t *mh, elem_t elem, unsigned int value);
int mh_remove_min(minheap_t *mh, elem_t* elem);
int mh_peek_min(minheap_t *mh, elem_t* elem);

#endif
