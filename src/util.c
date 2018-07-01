#include <util.h>
#include <debug.h>
#include <syscall.h>

void * memcpy(void * const dest, const void* const src, unsigned int sz){
    //*
    long * plDst = (long *) dest;
    long const * plSrc = (long const *) src;

    if (!((int)src & 0xFFFFFFFC) && !((int)dest & 0xFFFFFFFC))
    {
        //copy word by word, but use duffs device to unroll loop
        register int n = sz >> 4;
        switch (sz & 0xC) {
            do {
                    *plDst++ = *plSrc++;
        case 0xC:   *plDst++ = *plSrc++;
        case 0x8:   *plDst++ = *plSrc++;
        case 0x4:   *plDst++ = *plSrc++;
        case 0x0:   ;
            } while (n-- > 0);
        }

        sz &= 0x3;
    }

    char * pcDst = (char *) plDst;
    char const * pcSrc = (char const *) plSrc;

    while (sz--)
    {
        *pcDst++ = *pcSrc++;
    }

    return (dest);
    /*/

    unsigned char *dp_c = dest;
    const unsigned char *sp_c = src;
    if (sz < 4) {
        switch (sz) {
        case 3:     dp_c[2] = sp_c[2];
        case 2:     dp_c[1] = sp_c[1];
        case 1:     dp_c[0] = sp_c[0];
        case 0:     return dest;
        }
    }

    ASSERT(((int)dest & 0x3) == ((int)src & 0x3), "source pointer and destination pointer offsets from word boundaries don't match");

    const int word_off = (int)src & 0x3;
    switch (word_off) {
    //lower offset means there are MORE bytes before the next word boundary        
    case 1:     dp_c[2] = sp_c[2];
    case 2:     dp_c[1] = sp_c[1];
    case 3:     dp_c[0] = sp_c[0];
    default:     ;
    }
    sp_c = (const unsigned char *)(((int)sp_c + 3) & ~0x3);
    dp_c = (unsigned char *)(((int)dp_c + 3) & ~0x3);
    sz -= word_off;

    unsigned int *dp_i = (unsigned int *)dp_c;
    const unsigned int *sp_i = (const unsigned int *)sp_c;

    //burst mode (in batches of 4 words)
    for (; (sz >> 4) > 0; sz -= (1 << 4)) {
        __asm__ volatile (
            "ldmia %[src]!, {r3-r6}\n\t"
            "stmdb %[dest], {r3-r6}\n\t"
            : [src] "=r" (sp_i), [dest] "=r" (dp_i)
            :
            : "r3", "r4", "r5", "r6");
    }

    const int words_left = (sz >> 2) & 0x3;
    switch (words_left) {
    case 0x3:   dp_i[2] = sp_i[2];
    case 0x2:   dp_i[1] = sp_i[1];
    case 0x1:   dp_i[0] = sp_i[0];
    default:     ;
    }
    sp_i += words_left;
    dp_i += words_left;

    dp_c = (unsigned char *)dp_i;
    sp_c = (const unsigned char *)sp_i;

    switch (sz & 0x3) {
    case 3:     dp_c[2] = sp_c[2];
    case 2:     dp_c[1] = sp_c[1];
    case 1:     dp_c[0] = sp_c[0];
    default:     return dest;
    }
    return dest;
    */
}

void memswap(void *a, void *b, unsigned int sz) {
    // from https://github.com/qca/open-plc-utils/blob/master/tools/memswap.c
    register char * b1 = a;
    register char *b2 = b;
    if (a != b) while (sz--) {
        char b = *b1;
        *b1++=*b2;
        *b2++=b;
    }
}
