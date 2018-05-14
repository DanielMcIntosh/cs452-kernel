#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>

#define FOREVER for(;;)

int kernel_init(){
    
    __asm__(
        "ldr r0, =KERNEL_ENTRY_POINT\n\t"\
        "ldr r1, =0x28\n\t"\
        "str r0, [r1]");
    return 0;
}

typedef void task_t;
task_t* schedule(){

    return 0;
}

inline static void __attribute__((always_inline)) update_cspr_mode(const int mode){
    int reg;
    __asm__(
        "MRS %[reg], CPSR\n\t" // Get CPSR
        "BIC %[reg], %[reg], #0x1F\n\t"// Remove current mode
        "ORR %[reg], %[reg], %[mode]\n\t"// Substitute  // FIXME: This could be optimized.
        "MSR CPSR_c, %[reg]\n\t" // 
        : [reg] "=r" (reg)
        : [mode] "i" (mode));
}

int __attribute__((noinline)) activate(task_t* task){
    int ret;

    int sp, cpsr_usr, pc_usr; // TEMP: get from task later

    __asm__(
        "MSR SPSR_cxsf, %[cpsr_usr]" // Set SPSR to mov to CPSR once movs is called.
        :
        : [cpsr_usr] "r"(cpsr_usr)
    );
    __asm__(
        "stmdb sp!, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14,r15}\n\t" // Store Kernel State
        "mov %[pc_usr], lr\n\t" // set lr_svc = pc
        :
        : [pc_usr]"r"(pc_usr)
    );
    update_cspr_mode(0x1F);// System Mode
    __asm__(
        "ldmia sp, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14,r15}\n\t" // Reload User State from User Stack
    );
    update_cspr_mode(0x13);
    __asm__(
        "movs lr, pc\n\t" // s: update CSPR
     );

     __asm__(
        ".word 0xF7FFFFFF\n\t" // Undefined Instruction - shouldn't ever be hit.
        "KERNEL_ENTRY_POINT:\n\t":
    );

    update_cspr_mode(0x1F); // Go to System Mode
    __asm__(
        "mov %[sp], sp\n\t" // Save Stack Pointer
        "stmdb sp!, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14,r15}\n\t" // save user state on user stack
        :
        : [sp]"r"(sp)
    );
    update_cspr_mode(0x13); // Go back to Supervisor Mode
    __asm__(
        "ldr %[ret], [lr, #-4]":
            : [ret]"r"(ret));
    return ret & 0x00FFFFFF;// drop first 8 bits
}

int handle(int a){

    return a;
}

int test_minheap(){
    entry_t entries[400];
    minheap_t lmh;
    minheap_t* mh = &lmh;

    mh_init(mh, entries, 400);

    int i = 30;
    elem_t e = ELEM(&i);
    int err = mh_add(mh, e, 10);
    ASSERT(err == 0, "Error Adding");

    elem_t e2;
    err = mh_peek_min(mh, &e2);
    ASSERT(e.item == e2.item, "Incorrect Peek Value");

    int j = 40000;
    elem_t e3 = ELEM(&j);
    err = mh_add(mh, e3, 9);
    ASSERT(err == 0, "Error Adding");

    err = mh_remove_min(mh, &e2);
    ASSERT(e2.item == e3.item, "Wrong Minimum");
    ASSERT(*(int*)e2.item == j, "Wrong Value");

    err = mh_remove_min(mh, &e2);
    ASSERT(e2.item == e.item, "Wrong Minimum");
    ASSERT(*(int*)e2.item == i, "Wrong Value");
    return 0;
}
int main(){
    // test_minheap();
    kernel_init();
    FOREVER handle(activate(schedule()));
    Pass();
    bwputstr(COM2, "Passed?");

    return 0;
}
