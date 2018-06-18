#ifndef KERNEL_H
#define KERNEL_H

#define ASM_STACK_PUSH(x) "str " x ", [sp, #-4]!\n\t"
#define ASM_STACK_POP(x) "ldr " x ", [sp], #4\n\t"

#define FOREVER for(;;)
#define STACK_SPACE_SIZE 0x800000
#define NUM_TRAINS 80

#define DUMPR(x) "mov r0, #1\n\tmov r1, "x"\n\t bl bwputr\n\t"

#define NAME_FUT "FUT"

void task_stack_metric_printer(int terminaltid);

#endif
