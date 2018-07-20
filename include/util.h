#ifndef UTIL_H
#define UTIL_H

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef UINT_MAX
#define UINT_MAX ((unsigned int)-1)
#endif

#ifndef INT_MAX
#define INT_MAX (UINT_MAX / 2)
#endif

#define bool int

void memswap(void * a, void * b, unsigned int sz);
void * memcpy(void * dest, const void* src, unsigned int sz);
void *memset(void *s, int c, unsigned int n);
int fastintsqrt(int);

#ifndef MAX
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef MIN
#define MIN(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif

#ifndef CLAMP
#define CLAMP(X, Y, Z) (((X) > (Y)) ? (Y) : (((X) < (Z)) ? (Z) : (X)))
#endif

#ifndef ABS
#define ABS(X) (X > 0 ? X : -X)
#endif

#ifndef MOVING_AVERAGE
#define MOVING_AVERAGE(N, O, A) ((A)*(N)/100 + (100-(A))*(O)/100)
#endif

#ifndef MS_TO_S
#define MS_TO_S(ms) ((ms)/1000)
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#define array_dim(x) (sizeof(x) / sizeof(x[0]))
#define sizeof_field(type, field) (sizeof(((type *)0)->field))
#define field_dim(type, member) (sizeof_field(type, member) / sizeof_field(type, member[0]))

#define FOREVER for(;;)

#ifndef __has_attribute
    #define __has_attribute(x) 0
#endif

#if __has_attribute(fallthrough)
#define FALLTHROUGH __attribute__((fallthrough))
#else
#define FALLTHROUGH
#endif


#endif
