#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 1

#define PANIC(x) bwputstr(COM2, x);\
    return 1;

#define IS(x) #x
#define S(x) IS(x) 
#define ASSERT(x, y) \
if (!(x) && DEBUG)\
    PANIC("ASSERT FAILED: " S(x) "\r\nFUNCTION: " S(__func__) "\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n")

#endif
