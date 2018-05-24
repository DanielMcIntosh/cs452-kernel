#include <hashtable.h>
#include <util.h>
#include <err.h>
#include <debug.h>


static int hash(char * key) {
    int h = 0;
    while(*key != NULL)
        h += *key++;
    return h % HT_SIZE;
}

static unsigned long long c2ull(char * key){
    // TODO use long long instead of char *
    return *((unsigned long long*) key);
}

static int streq(char * a, char * b){
    while (*a != NULL && *b != NULL){
        if (*a++ != *b++)
            return 0;
    }
    if ((*a == NULL && *b != NULL) || 
        (*b == NULL && *a != NULL))
        return 0;

    return 1;
}

void ht_init(Hashtable *ht){
    for (int i = 0; i < HT_SIZE; i++){
        *((*ht)[i].key) = NULL;
        (*ht)[i].value = -1;
    }
}

int ht_insert(Hashtable *ht, char *key, int value){
    #if DEBUG
    bwprintf(COM2, "HT Insert: %d, %s, %d\r\n", ht, key, value);
    #endif
    int hsh = hash(key), n = 0;
    ASSERT(hsh < HT_SIZE, "Hash Invariant", ERR_INVARIANT_BROKEN);
    while ((*ht)[hsh].value != -1) {
        hsh++; n++;
        if (hsh >= HT_SIZE){
            hsh = 0;
        }
        if (n > HT_SIZE) {
            return ERR_HT_FULL;
        }
    }
    memcpy((*ht)[hsh].key, key, HT_KEY_SIZE);
    (*ht)[hsh].value = value;
    #if DEBUG
    bwprintf(COM2, "Hash: %d |-> Value: %d\r\n", hsh, value);
    #endif
    return 0;
}

int ht_lookup(Hashtable *ht, char * key){
    int hsh = hash(key), n = 0;
    ASSERT(hsh < HT_SIZE, "Hash Invariant", ERR_INVARIANT_BROKEN);
    while (streq((*ht)[hsh].key, key) == 0) {
        hsh++; n++;
        if (hsh >= HT_SIZE){
            hsh = 0;
        }
        if (n > HT_SIZE) {
            return ERR_HT_NOT_FOUND;
        }
    }
    bwprintf(COM2, "Hash: %d |-> Value: %d\r\n", hsh, (*ht)[hsh].value);
    return (*ht)[hsh].value;
}
