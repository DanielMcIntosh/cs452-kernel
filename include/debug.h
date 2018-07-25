#ifndef DEBUG_H
#define DEBUG_H

#include <ts7200.h>
#include <bwio.h>
#include <util.h>

#define DEBUG 0
#define DEBUG_COM2 0

#define PANIC(...) \
    EnterCriticalSection();\
    bwprintf(COM2, __VA_ARGS__);\
    Quit();

#define KPANIC(...) \
    bwprintf(COM2, __VA_ARGS__);\
    __builtin_trap();

#define IS(x) #x
#define S(x) IS(x) 
#define ASSERT(x, y,vargs...) \
if (unlikely(!(x))) {\
    PANIC("\033[0m\033[1;37;41mASSERT FAILED:\033[21;31;40m\r\n" S(x) "\r\nFUNCTION: %s\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n\033[0m", __func__,  ##vargs)\
}

#define KASSERT(x, y,vargs...) \
if (unlikely(!(x))) {\
    KPANIC("\033[0m\033[1;35;44mKASSERT FAILED:\033[21;31;40m\r\n" S(x) "\r\nFUNCTION: %s\r\nFILE: "S(__FILE__) "\r\nLINE: " S(__LINE__) "\r\n" S(y) "\r\n\033[0m", __func__,  ##vargs)\
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
