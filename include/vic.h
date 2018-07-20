#ifndef VIC_H
#define VIC_H

#define VIC_SIZE 32

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
    if (n >= VIC_SIZE)
        vic2->SoftInt |= 1U << (n - VIC_SIZE);
    else 
        vic1->SoftInt |= 1U << n;
}

#endif
