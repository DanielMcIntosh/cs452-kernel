#include <command.h>
#include <message.h>
#include <syscall.h>
#include <name.h>
#include <uart.h>
#include <debug.h>
#include <kernel.h>
#include <err.h>
#include <tasks.h>
#include <clock.h>

typedef struct commandmessage{
    MessageType type;
    Command command;
    // int deadline?
} CommandMessage;

typedef struct commandserver{
    int notifier_waiting;
    int additional_delay;
    

} CommandServer;

int SendCommand(int servertid, Command c){
    CommandMessage cm = {MESSAGE_COMMAND, c};
    ReplyMessage rm = {0, 0};

    int r = Send(servertid, &cm, sizeof(cm), &rm, sizeof(rm));
    return r >= 0 ? rm.ret : r;
}

void task_solenoid_off(){
    // TODO compress solenoid stuff into just one task? instead of spawning one per switch. At least, do only 1 at once.
    int tid = WhoIs(NAME_COMMANDSERVER);
    Delay(150);
    Command c = {COMMAND_NOTIFY_SOLENOID_TIMER, 0, 0};
    SendCommand(tid, c);
    Destroy();
}

void task_commandserver(){
    CommandServer cs = {0, 0};
    RegisterAs(NAME_COMMANDSERVER);
    int servertid = WhoIs(NAME_UART1_SEND), tid;//, uart2tid = WhoIs(NAME_UART2_SEND);
    CommandMessage cm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};

    FOREVER{
        Receive(&tid, &cm, sizeof(cm));
        if (cm.command.type >= INVALID_COMMAND){
            rm.ret = ERR_INVALID_COMMAND;
        } else {
            rm.ret = 0;
        }
        Reply(tid, &rm.ret, sizeof(rm));
        switch(cm.command.type) {
        case COMMAND_GO:
        {
            Putc(servertid, 1, 0x60);
            break;
        }
        case COMMAND_TR:
        {
            Putc(servertid, 1, cm.command.arg1);
            Putc(servertid, 1, cm.command.arg2);
            // TODO: update train statusses? (current plan: command server also holds train status info, can be queried in addition to sent commands. this might break SRP)
            break;
        }
        case COMMAND_RV:
        {
            Putc(servertid, 1, 0);
            Putc(servertid, 1, cm.command.arg1);
            // TODO: async timer reverse
            break;
        }
        case COMMAND_SW:
        {
            if (cs.notifier_waiting)
                cs.additional_delay=1; // TODO time server courier?
            else 
                cs.notifier_waiting = Create(PRIORITY_NOTIFIER, &task_solenoid_off);

            Putc(servertid, 1, cm.command.arg1 == 'C' ? 34 : 33);
            Putc(servertid, 1, cm.command.arg2);
            break;
        }
        case COMMAND_QUIT:
        {
            Quit(); // TODO
            break;
        }

        // partial commands
        case COMMAND_NOTIFY_RV_ACCEL:
        {
            //reaccelerate
            break;
        }
        case COMMAND_NOTIFY_RV_REVERSE:
        {
            Putc(servertid, 1, 15);
            Putc(servertid, 1, cm.command.arg1); 
            break;    
        }
        case COMMAND_NOTIFY_SOLENOID_TIMER:
            if (cs.additional_delay){
                cs.notifier_waiting = Create(PRIORITY_NOTIFIER, &task_solenoid_off);
                cs.additional_delay = 0;
            }
            else {
                Putc(servertid, 1, 32);
                cs.notifier_waiting = 0;
            }
            break;
        default:
        {
            PANIC("INVALID COMMAND UNCAUGHT");
        }
        }
    }
}
