#ifndef UTIL_H
#define UTIL_H

void memswap(void * a, void * b, unsigned int sz);
void * memcpy(void * dest, const void* src, unsigned int sz);

#ifndef NULL
#define NULL 0
#endif

#endif
