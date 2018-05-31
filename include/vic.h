#ifndef VIC_h
#define VIC_H

extern struct vic {
    volatile unsigned int IRQStatus: 8;
    volatile unsigned int RawIntr: 8;
    volatile unsigned int IntEnable: 4;
    volatile unsigned int IntEnableClear: 4;
    volatile unsigned int SoftInt: 4;
    volatile unsigned int SoftIntClear: 4;
} *vic1, *vic2;

#endif
