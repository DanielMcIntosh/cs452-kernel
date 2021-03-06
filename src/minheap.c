#include <minheap.h>
#include <util.h>
#include <debug.h>
#include <err.h>
#include <syscall.h>

void mh_init(minheap_t *mh, entry_t * entries, unsigned int size){
    mh->entries = entries;
    mh->size = size;
    mh->count = 0;
}

static void mh_bubble_up(minheap_t *mh, unsigned int idx) {
    entry_t e = mh->entries[idx];
    unsigned int parent_idx = (idx - 1) / 2;
    entry_t p = mh->entries[parent_idx];
    if (e.value < p.value) {
        memswap(mh->entries+idx, mh->entries+parent_idx, sizeof(entry_t));
        if (parent_idx > 0) {
            mh_bubble_up(mh, parent_idx);
        }
    }
}

static void mh_bubble_down(minheap_t *mh, unsigned int idx) {
    entry_t e = mh->entries[idx];
    unsigned int lc = (idx * 2) + 1;
    ASSERT(lc < mh->size, "there might be an actual problem here: %d %d %d", idx, lc, mh->count);
    unsigned int rc = lc+1;

    unsigned int swapped = 0;
    unsigned int swapped_idx;

    if (rc < mh->count  // right exists
        && mh->entries[rc].value < mh->entries[lc].value// less than left
        && e.value > mh->entries[rc].value){ // greater than right
        memswap(mh->entries+idx, mh->entries+rc, sizeof(entry_t));
        swapped = 1;
        swapped_idx = rc;
    } else if (e.value > mh->entries[lc].value) {
        memswap(mh->entries+idx, mh->entries+lc, sizeof(entry_t));
        swapped = 1;
        swapped_idx = lc;
    }
    if (swapped && (swapped_idx * 2) + 1 < mh->count -1) {
        mh_bubble_down(mh, swapped_idx);
    }
}

int mh_add(minheap_t *mh, unsigned long item, unsigned int value) {
    if (mh->count >= mh->size){
        return ERR_MH_FULL;
    }
    mh->entries[mh->count].item= item;
    mh->entries[mh->count++].value = value;
    if (mh->count > 1) {
        mh_bubble_up(mh, mh->count - 1);
    }
    return 0;
}

int mh_remove_min(minheap_t * restrict mh, entry_t * restrict min){
    ASSERT(min != 0, "Return ptr cannot be null");
    if (mh->count == 0) {
        return ERR_MH_EMPTY;
    }

    entry_t item;
    if (mh->count > 1){
        memswap(mh->entries, mh->entries+mh->count -1, sizeof(entry_t));
    }
    item = mh->entries[--mh->count];
    if (mh->count > 1) {
        mh_bubble_down(mh, 0);
    }

    *min = item;
    return 0;
}

int mh_peek_min(minheap_t * restrict mh, entry_t * restrict min){
    ASSERT(min != 0, "Return ptr cannot be null");
    if (mh -> count == 0){
        return ERR_MH_EMPTY;
    }

    *min = mh->entries[0];
    return 0;
}

int __attribute__((pure)) mh_empty(minheap_t *mh){
    return mh->count == 0;
}
