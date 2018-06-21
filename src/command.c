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
#include <terminal.h>
#include <circlebuffer.h>
#include <train.h>
#include <track_state.h>
#include <switch.h>

typedef struct commandmessage{
    MessageType type;
    Command command;
    // int deadline?
} CommandMessage;

typedef struct commandserver{
    int notifier_waiting;
    int solenoid_off;
    int courier_waiting;
    int courier_arg;
    int train_states[NUM_TRAINS];
    int train_speeds[NUM_TRAINS]; // TODO move this data to track_state.h?
    circlebuffer_t *cb_switches;
} CommandServer;

int SendCommand(int servertid, Command c){
    CommandMessage cm = {MESSAGE_COMMAND, c};
    ReplyMessage rm = {0, 0};

    int r = Send(servertid, &cm, sizeof(cm), &rm, sizeof(rm));
    return r >= 0 ? rm.ret : r;
}

static void init_switches(CommandServer *cs){
    ASSERT(cb_empty(cs->cb_switches), "Cannot init non-empty cb");

    char SWITCH_INIT_STATES[] = {
        [1] = 'C', 
        [2] = 'C',
        [3] = 'C',
        [4] = 'C',
        [5] = 'C',
        [6] = 'C',
        [7] = 'C',
        [8] = 'S',
        [9] = 'C',
        [10] = 'C',
        [11] = 'C',
        [12] = 'C',
        [13] = 'C',
        [14] = 'S',
        [15] = 'C',
        [16] = 'C',
        [17] = 'C',
        [18] = 'C',
        [153] = 'C',
        [154] = 'S',
        [155] = 'C',
        [156] = 'S',
        [157] = 'C',
        [158] = 'S',
        [159] = 'C',
        [160] = 'S'
    };

    for (int i = 1; i <= 18; i++){
        cb_write(cs->cb_switches, SWITCH_INIT_STATES[i]);
        cb_write(cs->cb_switches, i);
    }
    for (int y = 153; y <= 156; y++){
        cb_write(cs->cb_switches, SWITCH_INIT_STATES[y]);
        cb_write(cs->cb_switches, y);
    }
}

void task_solenoid_notifier(int tid){
    Command c = {COMMAND_NOTIFY_SOLENOID_TIMER, 0, 0};
    FOREVER {
        SendCommand(tid, c);
        Delay(17);
    }
}

int calcReverseTime(int speed){
    return 350 / (15 - speed) + 75;
}

void task_reverse_train(int arg){
    int train = arg >> 16;
    int speed = arg & 0xffff;
    int tid = WhoIs(NAME_COMMANDSERVER);

    Delay(calcReverseTime(speed));
    Command crv = {COMMAND_NOTIFY_RV_REVERSE, train, 15};
    SendCommand(tid, crv);
    Delay(10);
    Command cra = {COMMAND_NOTIFY_RV_ACCEL, speed, train};
    SendCommand(tid, cra);
    Destroy();
}

void task_switch_courier(int cmdtid){
    int terminaltid = WhoIs(NAME_TERMINAL);
    Command c = {COMMAND_NOTIFY_COURIER, 0, 0};
    FOREVER {
        int r = SendCommand(cmdtid, c);
        // unpack r
        int arg1 = r >> 16;
        int arg2 = r & 0xFFFF;
        SendTerminalRequest(terminaltid, TERMINAL_SWITCH, arg1, arg2);
    }
}

static inline void commandserver_exec_switch(CommandServer *cs, int arg1, int arg2, ReplyMessage *rm, int servertid){
    Putc(servertid, 1, arg1 == 'C' ? 34 : 33);
    Putc(servertid, 1, arg2);
    SwitchData sd = {arg1 == 'C' ? SWITCH_CURVED : SWITCH_STRAIGHT, arg2};
    NotifySwitchStatus(WhoIs(NAME_TRACK_STATE), sd); // TODO store tid value
    cs->solenoid_off = 0;
    if (cs->courier_waiting != 0){
        rm->ret = (arg1 << 16) | arg2;
        Reply(cs->courier_waiting, rm, sizeof(*rm));
        cs->courier_waiting = 0;
    } else if (cs->courier_arg == -1) {
        cs->courier_arg = (arg1 << 16) | arg2;
    }
}

void task_commandserver(){
    CommandServer cs = {0, 0, 0, -1, {0}, {0}, 0};
    RegisterAs(NAME_COMMANDSERVER);
    int servertid = WhoIs(NAME_UART1_SEND), tid;
    CommandMessage cm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    char switchQ_buf[SWITCHQ_BUF_SIZE];
    circlebuffer_t cb_switches;
    cb_init(&cb_switches, switchQ_buf, SWITCHQ_BUF_SIZE);
    cs.cb_switches = &cb_switches;

    CreateWithArgument(PRIORITY_NOTIFIER, task_solenoid_notifier, MyTid());
    Putc(servertid, 1, 0x60);
    init_switches(&cs);

    FOREVER {
        Receive(&tid, &cm, sizeof(cm));
        if (cm.command.type >= INVALID_COMMAND){
            rm.ret = ERR_INVALID_COMMAND;
        } else {
            rm.ret = 0;
        }
        if (cm.command.type != COMMAND_NOTIFY_COURIER && cm.command.type != COMMAND_NOTIFY_SOLENOID_TIMER) {
            Reply(tid, &rm.ret, sizeof(rm));
        }

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
            cs.train_speeds[cm.command.arg2] = cm.command.arg1; // update speed in server
            break;
        }
        case COMMAND_RV:
        {
            Putc(servertid, 1, 0);
            Putc(servertid, 1, cm.command.arg1);
            CreateWithArgument(PRIORITY_NOTIFIER, &task_reverse_train, (cm.command.arg1 << 16) | (cs.train_speeds[cm.command.arg1])); // TODO proper bit packed struct here
            break;
        }
        case COMMAND_SW:
        {
            if (cs.notifier_waiting) {
                Reply(cs.notifier_waiting, &rm, sizeof(rm));
                cs.notifier_waiting = 0;
                commandserver_exec_switch(&cs, cm.command.arg1, cm.command.arg2, &rm, servertid);
            } else {
                cb_write(cs.cb_switches, cm.command.arg1);
                cb_write(cs.cb_switches, cm.command.arg2);
            }
            break;
        }
        case COMMAND_QUIT:
        {
            Quit();
            break;
        }

        // partial commands
        case COMMAND_NOTIFY_RV_ACCEL:
        {
            //reaccelerate
            Putc(servertid, 1, cm.command.arg1);
            Putc(servertid, 1, cm.command.arg2);
            break;
        }
        case COMMAND_NOTIFY_RV_REVERSE:
        {
            Putc(servertid, 1, 15);
            Putc(servertid, 1, cm.command.arg1); 
            break;    
        }
        case COMMAND_NOTIFY_SOLENOID_TIMER:
        {
            if (cb_empty(cs.cb_switches) && !cs.solenoid_off){
                Putc(servertid, 1, 32);
                cs.solenoid_off = 1;
                Reply(tid, &rm, sizeof(rm));
            } else if (cb_empty(cs.cb_switches)) {
                cs.notifier_waiting = tid;
            } else {
                char arg1, arg2;
                int err;
                err = cb_read(cs.cb_switches, &arg1);
                ASSERT(err == 0, "CB SHOULD NOT BE EMPTY");
                err = cb_read(cs.cb_switches, &arg2);
                ASSERT(err == 0, "CB SHOULD NOT BE EMPTY");
                commandserver_exec_switch(&cs, arg1, arg2, &rm, servertid);
                Reply(tid, &rm, sizeof(rm));
            }
            break;
        }
        case COMMAND_SENSOR_REQUEST:
        {
            Putc(servertid, 1, 193 + cm.command.arg1); // individual sensor query TODO: does it make sense to do the full thing actually? since that uses less bandwidth
            break;
        }
        case COMMAND_NOTIFY_COURIER:
        {
            if (cs.courier_arg != -1){
                rm.ret = cs.courier_arg;
                Reply(tid, &rm, sizeof(rm));
                cs.courier_arg = -1;
            } else {
                cs.courier_waiting = tid;
            }
            break;
        }
        default:
        {
            PANIC("INVALID COMMAND UNCAUGHT");
        }
        }
    }
}
