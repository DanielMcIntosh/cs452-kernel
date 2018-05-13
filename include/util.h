#ifndef UTIL_H
#define UTIL_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

void memswap(void * a, void * b, unsigned int sz);
void * memcpy(void * dest, const void* src, unsigned int sz);

#endif
