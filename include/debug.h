#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0

#define PANIC(x) bwputstr(COM2, x);\
    __asm__("mov pc, #0");

#define IS(x) #x
#define S(x) IS(x) 
#define ASSERT(x, y, z) \
if (!(x) && DEBUG) {\
    PANIC("ASSERT FAILED: " S(x) "\r\nFUNCTION: " S(__func__) "\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n")\
}

#if DEBUG
#define LOGF(...) bwprintf(COM2, __VA_ARGS__);
#define LOGC(c) bwputc(COM2, (c));
#define LOG(str) bwputstr(COM2, str);
#else
#define LOGF(...)
#define LOGC(c)
#define LOG(str)
#endif


#endif
