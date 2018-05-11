#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 1

#define PANIC(x) bwputstr(COM2, x);\
    return 0;

#define I1(x) #x
#define S(x) I1(x) 
#define ASSERT(x, y) \
if (!x && DEBUG)\
    PANIC("ASSERT FAILED: " S(x) "\n FUNCTION: " S(__FUNCTION__) "\nLINE: " S(__LINE__) "\n" S(y))

#endif
