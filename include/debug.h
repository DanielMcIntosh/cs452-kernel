#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0
#define DEBUG_UART 0
#define bwio 1

#define PANIC(x) \
    EnterCriticalSection();\
    bwputstr(COM1, x);\
    Quit();

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

#if DEBUG_UART
#define DLOGC(c) if (uart == 1) Putc(WhoIs(NAME_UART2_SEND), c);
#define DLOG(str) if (uart == 1) Puts(WhoIs(NAME_UART2_SEND), #str, sizeof(#str));
#else
#define DLOGC(c)
#define DLOG(str)
#endif

#endif
