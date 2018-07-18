#include <hashtable.h>
#include <util.h>
#include <err.h>
#include <debug.h>
#include <syscall.h>


static __attribute__((pure)) int hash(char * key) {
    /*
    int h = 0;
    while(*key != NULL)
        h += *key++;
    return h % HT_SIZE;
    /*/
    unsigned long hash = 5381;
    int c;

    while ((c = *key++))
        hash = hash * 33 + c;

    return hash % HT_SIZE;
    //*/
}

/*
static unsigned long long c2ull(char * key){
    unsigned long long v = *((unsigned long long*) key);
    //clear the bytes after the null character (17 operations):
    //ascii values only use the lower 7 bits, so we don't have to worry about the high bit
    unsigned long long zeroBytes = ~((v + 0x7F7F7F7F7F7F7F7F) | 0x7F7F7F7F7F7F7F7F);
    unsigned long long mask = zeroBytes;
    mask |= mask >> 1;
    mask |= mask >> 2;
    mask |= mask >> 4;
    mask |= mask >> 8;
    mask |= mask >> 16;
    mask |= mask >> 32;
    mask = ~mask;

    return v & mask;
}
//*/

static int streq(char * a, char * b){
    for (int i = 0; i < HT_KEY_SIZE && *a != '\0' && *b != '\0'; ++i){
        if (*a++ != *b++)
            return 0;
    }
    return (*a != '\0' && *b != '\0') || (*a == *b);
}

void ht_init(Hashtable *ht){
    for (int i = 0; i < HT_SIZE; i++){
        memset((*ht)[i].key, '\0', HT_KEY_SIZE);
        (*ht)[i].value = -1;
    }
}

int ht_insert(Hashtable *ht, char key[static HT_KEY_SIZE], int value){
    LOGF("HT Insert: %d, %s, %d\r\n", ht, key, value);
    int hsh = hash(key), n = 0;
    ASSERT(hsh < HT_SIZE, "Hash Invariant");
    while ((*ht)[hsh].value != -1) {
        hsh++; n++;
        if (hsh >= HT_SIZE){
            hsh = 0;
        }
        ASSERT (n <= HT_SIZE, "HT INSERT FAILED: FULL");
    }
    memcpy((*ht)[hsh].key, key, HT_KEY_SIZE);
    (*ht)[hsh].value = value;
    LOGF("Hash: %d |-> Value: %d\r\n", hsh, value);
    return 0;
}

int ht_lookup(Hashtable *ht, char key[static HT_KEY_SIZE]){
    int hsh = hash(key), n = 0;
    ASSERT(hsh < HT_SIZE, "Hash Invariant");
    while (streq((*ht)[hsh].key, key) == 0) {
        hsh++; n++;
        if (hsh >= HT_SIZE){
            hsh = 0;
        }
        ASSERT (n <= HT_SIZE, "HT LOOKUP FAILED: %s NOT FOUND", key);
    }
    LOGF("Hash: %d |-> Value: %d\r\n", hsh, (*ht)[hsh].value);
    return (*ht)[hsh].value;
}

void ht_rev_lookup(Hashtable *ht, int value, char *result_buf) {
    for (int i = 0; i < HT_SIZE; ++i) {
        int cur_value = (*ht)[i].value;
        if (cur_value == value) {
            memcpy(result_buf, (*ht)[i].key, HT_KEY_SIZE);
        }
    }
}
