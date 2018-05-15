#include <debug.h>
#include <minheap.h>
#include <elem.h>
#include <kernel.h>
#include <tasks.h>
#include <bwio.h>

#define FOREVER for(;;)
#define STACK_SPACE_SIZE 1600

#define DUMPR(x) "mov r0, #1\n\tmov r1, "x"\n\t bl bwputr\n\t"

int kernel_init(){
    __asm__(
        "ldr r0, =KERNEL_ENTRY_POINT\n\t"\
        "ldr r1, =0x28\n\t"\
        "str r0, [r1]");
    return 0;
}

TD* schedule(){
    return task_nextActive();
}

inline static void __attribute__((always_inline)) update_cspr_mode(const int mode){
    __asm__(
        "MRS r0, CPSR\n\t" // Get CPSR
        "BIC r0, r0, #0x1F\n\t"// Remove current mode
        "ORR r0, r0, %[mode]\n\t"// Substitute  // FIXME: This could be optimized.
        "MSR CPSR_c, r0\n\t" // 
        : 
        : [mode] "i" (mode));
}

int __attribute__((noinline)) activate(TD *task){
    int ret;

    //int sp = task->sp, cpsr_usr = task->spsr, pc_usr = task->lr, task_ret=task->r0; // TEMP: get from task later
    int sp = task->sp, cpsr_usr = task->spsr, pc_usr=task->lr, task_ret;

    // KERNEL EXIT: ("before the context switch")
    __asm__(
        "MSR SPSR_cxsf, %[cpsr_usr]" // 5. Set SPSR_svc to CPSR_user, which will return it to user mode once movs is called.
        :
        : [cpsr_usr] "r"(cpsr_usr)
    );
    update_cspr_mode(0x1F);// 9. Switch to System Mode
    __asm__(
        "mov sp, %[sp]\n\t" // Reload Stack Pointer
        "ldmia sp, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14}\n\t" // 10. Reload registers from User Stack
        :
        : [sp]"r"(sp));
    update_cspr_mode(0x13); // 11. Switch to Supervisor mode.
    __asm__(
        "mov lr, %[lr]\n\t"
        :
        :[lr]"r"(pc_usr)
    );
    __asm__(
        DUMPR("r4")
        DUMPR("r5")
        DUMPR("r6")
        DUMPR("r7")
        DUMPR("r8")
        DUMPR("r9")
        DUMPR("r10")
        DUMPR("r11")
        DUMPR("r12")
        DUMPR("r13")
        DUMPR("r14")
    );
    __asm__(
        //"mov r0, %[task_ret]\n\t"
        "movs pc, lr\n\t" // 13. s: update CSPR
        :
        ://[task_ret]"r"(task_ret)
     );

     __asm__(
        ".word 0xF7FFFFFF\n\t" // Undefined Instruction - shouldn't ever be hit.
        "KERNEL_ENTRY_POINT:\n\t":
    );

    // KERNEL ENTER: ("after the context switch")

    // Acquire Args
    /*
    int a1,a2,a3,a4;
    __asm__(
        "mov %[a1], r0\n\t"
        "mov %[a2], r1\n\t"
        "mov %[a3], r2\n\t"
        "mov %[a4], r3\n\t"
        : [a1]"=r"(a1),[a2]"=r"(a2),[a3]"=r"(a3),[a4]"=r"(a4)
        :
        : "r0","r1","r2","r3"
    );
    */

    __asm__(
        DUMPR("r4")
        DUMPR("r5")
        DUMPR("r6")
        DUMPR("r7")
        DUMPR("r8")
        DUMPR("r9")
        DUMPR("r10")
        DUMPR("r11")
        DUMPR("r12")
        DUMPR("r13")
        DUMPR("r14")
    );
    // Acquire PC of active task
    __asm__(
        "mov %[lr], lr\n\t"
        DUMPR("sp")
        : [lr]"=r"(pc_usr)
    );

    update_cspr_mode(0x1F); // Go to System Mode
    __asm__(
        DUMPR("sp")
        "stmdb sp!, {r4,r5,r6,r7,r8,r9,r10,r11,r12,r14}\n\t" // Push user registers onto active task stack
        "mov %[sp], sp\n\t" // Save Stack Pointer
        : [sp]"=r"(sp)
    );
    update_cspr_mode(0x13); // Go back to Supervisor Mode
    
    __asm__(
        DUMPR("sp")
        "MRS %[cpsr_usr], SPSR"
        : [cpsr_usr]"=r"(cpsr_usr)
     );
    __asm__(
        "ldr %[ret], [lr, #-4]":
            : [ret]"r"(ret)
    );

    task->sp = sp;
    task->lr = pc_usr;
    task->spsr = cpsr_usr;

    return ret & 0x00FFFFFF;// drop first 8 bits
}

int handle(int a, TD *task){
    switch(a) {
        case SYSCALL_CREATE:
        {
            break;
        }
        case SYSCALL_TID:
        {
            task->r0 = task_getTid(task);
            break;
        }
        case SYSCALL_PTID:
        {
            task->r0 = task_getParentTid(task);
            break;
        }
        case SYSCALL_PASS:
        {
            break;
        }
        case SYSCALL_EXIT:
        {
            break;
        }
        default:
        {
            break;
        }
    }
    return a;
}

void fak(){
    FOREVER {
        bwputstr(COM1, "#believe");
        Pass();
    }

}

int main(){
    bwputstr(COM1, "Start!");
    kernel_init();
    bwputstr(COM1, "Kernel Init!");
    TD task_pool[TASK_POOL_SIZE];
    TD *ready_queues[NUM_PRIORITY];
    char stack_space[STACK_SPACE_SIZE];
    bwputstr(COM1, "Stack Space!");
    task_init(task_pool, stack_space, STACK_SPACE_SIZE);
    bwputstr(COM1, "Task Init!");
    task_create(0, PRIORITY_HIGHEST, (int) &fak);
    bwputstr(COM1, "Task Created!");
    bwprintf(COM1, "&fak: %p", &fak);

    //FOREVER 
    for (int i = 0; i < 4; i++){
        TD *task = schedule();
        bwputstr(COM1, "Task Scheduled!");
        handle(activate(task), task);
    }
    bwputstr(COM2, "Passed?");

    return 0;
}
