#ifndef MINHEAP_H
#define MINHEAP_H

typedef struct entry {
    unsigned int item;
    unsigned int value;
} entry_t;

typedef struct minheap {
    entry_t* entries;
    unsigned int count;
    unsigned int size;
} minheap_t;

void mh_init(minheap_t *mh, entry_t * entries, unsigned int size);
int mh_add(minheap_t *mh, int item, unsigned int value);
int mh_remove_min(minheap_t *mh, entry_t* min);
int mh_peek_min(minheap_t *mh, entry_t* min);

#endif
