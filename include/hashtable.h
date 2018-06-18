#ifndef HASHTABLE_H
#define HASHTABLE_H

#define HT_EXPECTED_MAX 100
#define HT_SIZE_COEFF 2
#define HT_SIZE (HT_SIZE_COEFF * HT_EXPECTED_MAX)
#define HT_KEY_SIZE 8

typedef struct ht_node{
    //unsigned long long key; // TODO
    char key[HT_KEY_SIZE];
    int value;
} HT_Node;

typedef HT_Node Hashtable[HT_SIZE];

void ht_init(Hashtable *ht);
int ht_insert(Hashtable *ht, char key[static HT_KEY_SIZE], int value);
int ht_lookup(Hashtable *ht, char key[static HT_KEY_SIZE]);
void ht_rev_lookup(Hashtable *ht, int value, char *result_buf);

#endif
