#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0
#define DEBUG_COM2 0

#define PANIC(...) \
    EnterCriticalSection();\
    bwprintf(COM2, __VA_ARGS__);\
    Quit();

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

#if DEBUG_COM2
#define CLOGC(c) bwputc(COM1, (c));
#define CLOGF(...) bwputc(COM1, __VA_ARGS__);
#define CLOG(str) bwputc(COM1, str);
#else
#define CLOGC(c)
#define CLOGF(...)
#define CLOG(str)
#endif

#endif
