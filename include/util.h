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

#ifndef NULL
#define NULL 0
#endif

#ifndef MAX
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef MIN
#define MIN(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif


#endif
