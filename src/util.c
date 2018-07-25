#include <util.h>

void * memcpy(void * dest, const void* src, unsigned int sz){
    // from http://clc-wiki.net/wiki/C_standard_library:string.h:memcpy#Implementation
    unsigned char *dp = dest;
    const unsigned char *sp = src;

    //use duffs device to unroll loop
    register unsigned int n = sz >> 2;
    switch (sz & 0x3) {
        do {
                {*dp++ = *sp++;  FALLTHROUGH;}
    case 3:     {*dp++ = *sp++;  FALLTHROUGH;}
    case 2:     {*dp++ = *sp++;  FALLTHROUGH;}
    case 1:     {*dp++ = *sp++;  FALLTHROUGH;}
    case 0:     ;
        } while (n-- > 0);
    }
    return dest;
}

void memswap(void *a, void *b, unsigned int sz) {
    // from https://github.com/qca/open-plc-utils/blob/master/tools/memswap.c
    register char * b1 = a;
    register char *b2 = b;
    if (a != b) {
        while (sz--) {
            char tmp = *b1;
            *b1++=*b2;
            *b2++=tmp;
        }
    }
}

void *memset(void *s, int c, unsigned int n) {
    unsigned char *p = s;
    while(n --> 0) {  
        __asm__(""); // ensure that we don't optimize memset to memset
        *p++ = (unsigned char)c;
    }
    return s;
}

int __attribute__((const)) fastintsqrt(int s) {
    int op  = s;
    int res = 0;
    int one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

    // "one" starts at the highest power of four <= than the argument.
    one >>= __builtin_clz(s) & 0x03;
    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;

}
