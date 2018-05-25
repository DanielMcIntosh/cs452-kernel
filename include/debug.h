#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0

#define PANIC(x, z) bwputstr(COM2, x);\
    return z;

#define IS(x) #x
#define S(x) IS(x) 
#define ASSERT(x, y, z) \
if (!(x) && DEBUG) {\
    PANIC("ASSERT FAILED: " S(x) "\r\nFUNCTION: " S(__func__) "\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n", z)\
}

#endif
