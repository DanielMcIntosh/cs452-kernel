#ifndef VIC_H
#define VIC_H

extern struct vic {
    volatile unsigned int IRQStatus: 32;
    volatile unsigned int FIQStatus: 32;
    volatile unsigned int RawIntr: 32;
    volatile unsigned int IntSelect: 32;
    volatile unsigned int IntEnable: 32;
    volatile unsigned int IntEnableClear: 32;
    volatile unsigned int SoftInt: 32;
    volatile unsigned int SoftIntClear: 32;
} *vic1, *vic2;

static inline void __attribute__((always_inline)) software_interrupt(int n){
    if (n > 32)
        vic2->SoftInt |= 1 << (n - 33);
    else 
        vic1->SoftInt |= 1 << (n - 1);
}

#endif
