#ifndef HASHTABLE_H
#define HASHTABLE_H

#define HT_EXPECTED_MAX 100;
#define HT_SIZE_COEFF 2;
#define HT_SIZE HT_SIZE_COEFF * HT_EXPECTED_MAX;

typedef Hashtable int[HT_SIZE_COEFF];

int ht_insert(Hashtable *ht, char * key, int value);
int ht_lookup(Hashtable *ht, char * key);

#endif
