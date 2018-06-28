#include <util.h>
#include <debug.h>
#include <syscall.h>

void * memcpy(void * dest, const void* src, unsigned int sz){
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

    switch ((int)src & 0x3) {
    //lower offset means there are MORE bytes before the next word boundary        
    case 1:     *dp_c++ = *sp_c++;
    case 2:     *dp_c++ = *sp_c++;
    case 3:     *dp_c++ = *sp_c++;
    case 0:     ;
    }
    sz -= (int)src & 0x3;
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

    switch (sz & 0xC) {
    case 0xC:   *dp_i++ = *sp_i++;
    case 0x8:   *dp_i++ = *sp_i++;
    case 0x4:   *dp_i++ = *sp_i++;
    case 0:     ;
    }    

    dp_c = dp_i;
    sp_c = sp_i;

    switch (sz & 0x3) {
    case 3:     *dp_c++ = *sp_c++;
    case 2:     *dp_c++ = *sp_c++;
    case 1:     *dp_c++ = *sp_c++;
    case 0:     return dest;
    }
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
