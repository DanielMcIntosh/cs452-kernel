#include <util.h>

void * memcpy(void * dest, const void* src, unsigned int sz){
    // from http://clc-wiki.net/wiki/C_standard_library:string.h:memcpy#Implementation
    unsigned char *dp = dest;
    const unsigned char *sp = src;

    //use duffs device to unroll loop
    register int n = sz >> 2;
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
