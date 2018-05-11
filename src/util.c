#include <util.h>

void * memcpy(void * dest, const void* src, unsigned int sz){
    // from http://clc-wiki.net/wiki/C_standard_library:string.h:memcpy#Implementation
    char *dp = dest;
    const char *sp = src;
    while (sz--)
        *dp++ = *sp++;
    return dest;
}

void memswap(void *a, void *b, unsigned int sz) {
    // from https://github.com/qca/open-plc-utils/blob/master/tools/memswap.c
    register char * b1 = (char *) a;
    register char *b2 = (char *) b;
    if (a != b) while (sz--) {
        char b = *b1;
        *b1++=*b2;
        *b2++=b;
    }
}
