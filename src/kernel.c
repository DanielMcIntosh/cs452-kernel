#include <debug.h>
#include <minheap.h>
#include <elem.h>


int main(){
    entry_t entries[400];
    minheap_t lmh;
    minheap_t* mh = &lmh;

    mh_init(mh, entries, 400);

    int i = 30;
    elem_t e = ELEM(&i);
    int err = mh_add(mh, e, 10);
    ASSERT(err == 0, "Error Adding");

    elem_t e2;
    err = mh_peek_min(mh, &e2);
    ASSERT(e.item == e2.item, "Incorrect Peek Value");

    int j = 40000;
    elem_t e3 = ELEM(&j);
    err = mh_add(mh, e3, 9);
    ASSERT(err == 0, "Error Adding");

    err = mh_remove_min(mh, &e2);
    ASSERT(e2.item == e3.item, "Wrong Minimum");
    ASSERT(*(int*)e2.item == j, "Wrong Value");

    err = mh_remove_min(mh, &e2);
    ASSERT(e2.item == e.item, "Wrong Minimum");
    ASSERT(*(int*)e2.item == i, "Wrong Value");

    return 0;
}
