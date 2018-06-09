#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0
#define bwio 1

#define PANIC(x) bwputstr(COM1, x);\
    __asm__("swi #0x123");\
    //__asm__("mov pc, #0x00");

#if !(bwio)
#define bwputc(...)
#define bwputstr(...)
#define bwprintf(...)
#endif


#define IS(x) #x
#define S(x) IS(x) 
#define ASSERT(x, y) \
if (!(x) && DEBUG) {\
    PANIC("ASSERT FAILED: " S(x) "\r\nFUNCTION: " S(__func__) "\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n")\
}

#if DEBUG
#define LOGF(...) bwprintf(COM1, __VA_ARGS__);
#define LOGC(c) bwputc(COM1, (c));
#define LOG(str) bwputstr(COM1, str);
#else
#define LOGF(...)
#define LOGC(c)
#define LOG(str)
#endif


#endif
