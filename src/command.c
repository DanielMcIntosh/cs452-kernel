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
#include <train_event.h>
#include <switch.h>
#include <util.h>

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

    const char SWITCH_INIT_STATES[] = {
        [1] = 'C', 
        [2] = 'C',
        [3] = 'C',
        [4] = 'C',
        [5] = 'C',
        [6] = 'C',
        [7] = 'S',
        [8] = 'S',
        [9] = 'C',
        [10] = 'C',
        [11] = 'C',
        [12] = 'C',
        [13] = 'S',
        [14] = 'C',
        [15] = 'C',
        [16] = 'C',
        [17] = 'S',
        [18] = 'C',
        [153] = 'C',
        [154] = 'S',
        [155] = 'C',
        [156] = 'S',
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
    Command c = {COMMAND_NOTIFY_SOLENOID_TIMER, 0, {.arg2 = 0}};
    FOREVER {
        SendCommand(tid, c);
        Delay(17);
    }
}

int calcReverseTime(int speed){
    return 350 / (15 - speed) + 75;
}

void task_reverse_train(int train, int speed){
    int tid = WhoIs(NAME_COMMANDSERVER), terminaltid = WhoIs(NAME_TERMINAL);
    Delay(calcReverseTime(speed));
    Command crv = {COMMAND_NOTIFY_RV_REVERSE, train, {.arg2 = 15}};
    SendCommand(tid, crv);
    Delay(10);
    Command cra = {COMMAND_NOTIFY_RV_ACCEL, speed, {.arg2 = train}};
    SendCommand(tid, cra);
    SendTerminalRequest(terminaltid, TERMINAL_FLAGS_UNSET, STATUS_FLAG_REVERSING, 0);
    Destroy();
}

void task_switch_courier(int cmdtid, int terminaltid){
    Command c = {COMMAND_NOTIFY_COURIER, 0, {.arg2 = 0}};
    FOREVER {
        int r = SendCommand(cmdtid, c);
        // unpack r
        int arg1 = r >> 16;
        int arg2 = r & 0xFFFF;
        SendTerminalRequest(terminaltid, TERMINAL_SWITCH, arg1, arg2);
    }
}

void task_short_move(int train, int delay){
    int tid = WhoIs(NAME_COMMANDSERVER);
    Delay(delay);
    Command c = {COMMAND_TR, 0, {.arg2 = train}};
    SendCommand(tid, c);
    Destroy();
}
    

static inline void commandserver_exec_switch(CommandServer * restrict cs, int arg1, int arg2, ReplyMessage * restrict rm, int servertid, int trackstate_tid){
    Putc(servertid, 1, arg1 == 'C' ? 34 : 33);
    Putc(servertid, 1, arg2);
    SwitchData sd = {arg1 == 'C' ? SWITCH_CURVED : SWITCH_STRAIGHT, arg2};
    NotifySwitchStatus(trackstate_tid, sd);
    cs->solenoid_off = 0;
    if (cs->courier_waiting != 0){
        rm->ret = (arg1 << 16) | arg2;
        Reply(cs->courier_waiting, rm, sizeof(*rm));
        cs->courier_waiting = 0;
    } else if (cs->courier_arg == -1) {
        cs->courier_arg = (arg1 << 16) | arg2;
    }
}

static inline void commandserver_exec_reverse(int train, int speed, int servertid, int trainstate_tid, int terminaltid) {
    Putc(servertid, 1, 0);
    Putc(servertid, 1, train);
    SendTerminalRequest(terminaltid, TERMINAL_FLAGS_SET, STATUS_FLAG_REVERSING, 0);
    CreateWith2Args(PRIORITY_NOTIFIER, &task_reverse_train, train, speed);
    TrainData td = {0, train};
    NotifyTrainSpeed(trainstate_tid, td); // TODO train state courier
}
//cannot be used from within the commandserver task
static inline void setup_track_state(int cmdtid, NavigateResult *nav_res) {
    for (int i = 1; i <= NUM_SWITCHES; i++) {
        if (nav_res->switches[i] != SWITCH_UNKNOWN){
            char sw_state = STATE_TO_CHAR(nav_res->switches[i]);
            int sw_num = SWUNCLAMP(i);
            Command cmd = {COMMAND_SW, sw_state, {.arg2 = sw_num}};
            SendCommand(cmdtid, cmd);
        }
    }

}

void send_wakeup(int tid, int __attribute__((unused)) arg1, bool success) {
    Send(tid, &success, sizeof(success), NULL, 0);
}

void task_calibrate(int train, int sensor_dest) {
    int cmdtid = WhoIs(NAME_COMMANDSERVER);
    int terminaltid = WhoIs(NAME_TERMINAL);
    int trainstate_tid = WhoIs(NAME_TRAIN_STATE);
    int my_tid = MyTid();

    SendTerminalRequest(terminaltid, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING | STATUS_FLAG_CALIBRATING, 0);

    NavigateResult nav_res = {0, 0, 0, {}};
    int alarm_tid;
    bool runnable_success;
    int speed = GetTrainSpeed(trainstate_tid, train);
    bool overshot;

    int num_over = 0;
    while (num_over == 0 || num_over == CAL_ITERATIONS) {
        num_over = 0;
        for (int i = 0; i < CAL_ITERATIONS; ++i) {
            //set the train on course to hit sensor_dest, and get timing info
            const TrackPosition pos = {.object = sensor_dest, .distance_past = 0};
            const NavigateRequest rq = {.position = pos, .train = train};
            int err = NavigateTo(trainstate_tid, rq, &nav_res);
            ASSERT(err==0, "FAILED TO GET ROUTE");
            //if (i < 2)
                setup_track_state(cmdtid, &nav_res);

            int sensor_to_wake = nav_res.end_sensor;
            //wait until we hit <sensor_to_wake>
            //for now, assume we're always successful in triggering <sensor_to_wake>
            Runnable runnable_alarm1 = {&send_wakeup, my_tid, 0, 50000U, FALSE};
            RunWhen(sensor_to_wake, &runnable_alarm1, PRIORITY_MID);
            Receive(&alarm_tid, &runnable_success, sizeof(runnable_success));
            Reply(alarm_tid, NULL, 0);
            if (!runnable_success)
            {
                continue;
            }

            int delay = nav_res.time_after_end_sensor;
            //wait <delay> ticks, then send a stop command
            Delay(delay);
            Command stop_cmd = {COMMAND_TR, 0, {.arg2 = train}};
            SendCommand(cmdtid, stop_cmd);

            //sleep until we hit sensor_dest, timeout 2.5 s after we send stop command
            Runnable runnable_alarm2 = {&send_wakeup, my_tid, 0, 250U, TRUE};
            RunWhen(sensor_dest, &runnable_alarm2, PRIORITY_MID);
            Receive(&alarm_tid, &overshot, sizeof(overshot));
            Reply(alarm_tid, NULL, 0);

            CalData cal_data = {i, speed, overshot};
            NotifyCalibrationResult(trainstate_tid, cal_data);

            Command accel_cmd = {COMMAND_TR, speed, {.arg2 = train}};
            SendCommand(cmdtid, accel_cmd);

            if (overshot) {
                ++num_over;
            }
        }
    }
    SendTerminalRequest(terminaltid, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING | STATUS_FLAG_CALIBRATING, 0);
    Destroy();
}

void stop_wrapper(int train, int wait, bool __attribute__((unused)) success) {
    int cmdtid = WhoIs(NAME_COMMANDSERVER);
    int terminaltid = WhoIs(NAME_TERMINAL);

    if (success) {
        Delay(wait);
        Command stop_cmd = {COMMAND_TR, 0, {.arg2 = train}};
        SendCommand(cmdtid, stop_cmd);
    }

    SendTerminalRequest(terminaltid, TERMINAL_FLAGS_UNSET, STATUS_FLAG_FINDING, 0);
}

void task_commandserver(int trackstate_tid, int trainstate_tid){
    CommandServer cs = {0, 0, 0, -1, 0};
    RegisterAs(NAME_COMMANDSERVER);
    int servertid = WhoIs(NAME_UART1_SEND), tid;
    int terminaltid = WhoIs(NAME_TERMINAL);
    CommandMessage cm;
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    NavigateResult nav_res = {0, 0, 0, {}};
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
            int speed = cm.command.arg1;
            int train = cm.command.arg2;
            Putc(servertid, 1, speed);
            Putc(servertid, 1, train);
            TrainData td = {speed, train};
            NotifyTrainSpeed(trainstate_tid, td);
            break;
        }
        case COMMAND_RV:
        {
            int train = cm.command.arg1;
            int speed = GetTrainSpeed(trainstate_tid, train);
            commandserver_exec_reverse(train, speed, servertid, trainstate_tid, terminaltid);
            break;
        }
        case COMMAND_SW:
        {
            char dir = cm.command.arg1;
            int sw = cm.command.arg2;
            if (cs.notifier_waiting) {
                Reply(cs.notifier_waiting, &rm, sizeof(rm));
                cs.notifier_waiting = 0;
                commandserver_exec_switch(&cs, dir, sw, &rm, servertid, trackstate_tid);
            } else {
                cb_write(cs.cb_switches, dir);
                cb_write(cs.cb_switches, sw);
            }
            break;
        }
        case COMMAND_INV:
        {
            int i = 1;
            if (cs.notifier_waiting) {
                Reply(cs.notifier_waiting, &rm, sizeof(rm));
                cs.notifier_waiting = 0;
                commandserver_exec_switch(&cs, INV_STATE_TO_CHAR(GetSwitchState(trackstate_tid, i)), i, &rm, servertid, trackstate_tid);
                i++;
            } 

            for (; i <= 18; i++) {
                cb_write(cs.cb_switches, INV_STATE_TO_CHAR(GetSwitchState(trackstate_tid, i)));
                cb_write(cs.cb_switches, i);
            }
            for (int y = 153; y <= 156; y++) {
                cb_write(cs.cb_switches, INV_STATE_TO_CHAR(GetSwitchState(trackstate_tid, y)));
                cb_write(cs.cb_switches, y);
            }
            break;
        }
        case COMMAND_ROUTE:
        {
            SendTerminalRequest(terminaltid, TERMINAL_FLAGS_SET, STATUS_FLAG_FINDING, 0);

            int sensor = cm.command.arg1;
            int distance_past = cm.command.smallarg1;
            int train = cm.command.smallarg2;
            const TrackPosition pos = {.object = SENSOR_TO_NODE(sensor), .distance_past = distance_past};
            const NavigateRequest rq = {.position = pos, .train = train};
            int err = NavigateTo(trainstate_tid, rq, &nav_res);
            ASSERT(err==0, "FAILED TO GET ROUTE");

            int time_after_sensor = nav_res.time_after_end_sensor;
            int sensor_to_wake = nav_res.end_sensor;
            //PANIC("%d || %d || %d", sensor, sensor_to_wake, time_after_sensor); //122 | 62 | 178
            for (int i = 1; i <= NUM_SWITCHES; i++) {
                if (nav_res.switches[i] != SWITCH_UNKNOWN){
                    char sw_state = STATE_TO_CHAR(nav_res.switches[i]);
                    int sw_num = SWUNCLAMP(i);
                    if (cs.notifier_waiting) {
                        Reply(cs.notifier_waiting, &rm, sizeof(rm));
                        cs.notifier_waiting = 0;
                        commandserver_exec_switch(&cs, sw_state, sw_num, &rm, servertid, trackstate_tid);
                    }  else {
                        cb_write(cs.cb_switches, sw_state);
                        cb_write(cs.cb_switches, sw_num); 
                    }
                }
            }
            if (nav_res.speed != CURRENT_SPEED) {
                if (nav_res.speed < 0) {
                    commandserver_exec_reverse(train, -1 * nav_res.speed, servertid, trainstate_tid, terminaltid);
                } else {
                    Putc(servertid, 1, nav_res.speed); 
                    Putc(servertid, 1, train);
                }
            }
            Runnable runnable = {&stop_wrapper, train, time_after_sensor, 3000U, TRUE};
            RunWhen(sensor_to_wake, &runnable, PRIORITY_MID);
            break;
        }
        case COMMAND_MOVE:
        {
            int train = cm.command.arg2;
            int distance = cm.command.arg1;
            ShortMessage sm;

            GetShort(trackstate_tid, distance, &sm); // TODO track state courier

            Putc(servertid, 1, sm.speed);
            Putc(servertid, 1, train);
            TrainData td = {train, sm.speed};
            NotifyTrainSpeed(trainstate_tid, td);
            CreateWith2Args(PRIORITY_LOW, &task_short_move, train, sm.delay);

            break;
        }
        case COMMAND_PARAM:
        {
            ParamData data = {.key = cm.command.smallarg1, .param = cm.command.smallarg2, .value = cm.command.arg1};
            NotifyParam(trackstate_tid, data); // TODO track state courier
            break;
        }
        case COMMAND_CAL:
        {
            int sensor = cm.command.arg1;
            int train = cm.command.arg2;
            CreateWith2Args(PRIORITY_MID, &task_calibrate, train, sensor);
            break;
        }
        case COMMAND_ADD:
        {
            int train = cm.command.arg1;
            int sensor = cm.command.arg2;
            NewTrain new = {.train = train, .sensor = sensor};
            NotifyNewTrain(trainstate_tid, new);
            break;
        }
        case COMMAND_RESERVE:
        {
            NotifyReservation(trainstate_tid, cm.command.arg1);
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
            int speed = cm.command.arg1;
            int train = cm.command.arg2;
            //reaccelerate
            Putc(servertid, 1, speed);
            Putc(servertid, 1, train);
            TrainData td = {speed, train};
            NotifyTrainSpeed(trainstate_tid, td);
            break;
        }
        case COMMAND_NOTIFY_RV_REVERSE:
        {
            int train = cm.command.arg1;
            Putc(servertid, 1, 15);
            Putc(servertid, 1, train); 
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
                commandserver_exec_switch(&cs, arg1, arg2, &rm, servertid, trackstate_tid);
                Reply(tid, &rm, sizeof(rm));
            }
            break;
        }
        case COMMAND_SENSOR_REQUEST:
        {
            // individual sensor query TODO: does it make sense to do the full thing actually? since that uses less bandwidth
            Putc(servertid, 1, 193 + cm.command.arg1);
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
