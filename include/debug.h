#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>

#define DEBUG 0
#define DEBUG_UART 0
#define bwio 1

#define PANIC(...) \
    EnterCriticalSection();\
    bwprintf(COM3, __VA_ARGS__);\
    Quit();

#if !(bwio)
#define bwputc(...)
#define bwputstr(...)
#define bwprintf(...)
#endif

#define IS(x) #x
#define S(x) IS(x)

#if DEBUG
#define ASSERT(x, y) \
if (!(x)) {\
    PANIC("ASSERT FAILED: " S(x) "\r\nFUNCTION: " S(__func__) "\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n")\
}
#else
#define ASSERT(x, y)
#endif

#if DEBUG
#define LOGF(...) bwprintf(COM3, __VA_ARGS__);
#define LOGC(c) bwputc(COM3, (c));
#define LOG(str) bwputstr(COM3, str);
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
