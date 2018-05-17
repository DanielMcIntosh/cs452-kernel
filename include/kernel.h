#ifndef KERNEL_H
#define KERNEL_H

#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4]!\n\t"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4\n\t"

#endif
