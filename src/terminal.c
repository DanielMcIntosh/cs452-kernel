#include <kernel.h>
#include <circlebuffer.h>
#include <terminal.h>
#include <name.h>
#include <uart.h>
#include <err.h>
#include <debug.h>
#include <command.h>
#include <syscall.h>
#include <tasks.h>
#include <message.h>

typedef struct terminalparser {
    circlebuffer_t input;
    Command cmd;
} TerminalParser;

typedef struct terminal {
    circlebuffer_t output;
    int input_line;
    int input_col;
    int sensor_line;
    int notifier;
} Terminal;

typedef struct terminalmessage {
    MessageType type;
    TerminalRequest rq;
    int arg1;
    int arg2;
} TerminalMessage;

static inline void cursor_to_position(struct circlebuffer *cb, int line, int col) {
    char * s = "\033[";
    cb_write_string(cb, s);
    cb_write_number(cb, line, 10);
    cb_write(cb, ';');
    cb_write_number(cb, col, 10);
    cb_write(cb, 'H');
}

static void output_base_terminal(Terminal *t) {
    circlebuffer_t *cb = &t->output;
    cb_write_string(cb, "\033[2J\033[0;0H");
    int i;
    for (i = 0; i < TERMINAL_INPUT_MAX_COL+1; i++){
        cb_write(cb, '=');
    }
    cursor_to_position(cb, t->input_line, t->input_col);
    cb_write_string(cb, "::> ");
    t->input_col+= 4;
    cursor_to_position(cb, 27, 1);
    cb_write_string(cb, "MAX : ");
    cursor_to_position(cb, 28, 1);
    cb_write_string(cb, "LMAX: ");
    cursor_to_position(cb, 29, 1);
    cb_write_string(cb, "AVG : ");
    cursor_to_position(cb, 30, 1);
    cb_write_string(cb, "SNSR: ");
    cursor_to_position(cb, 31, 1);
    cb_write_string(cb, "FSNR: ");

    cursor_to_position(cb, 4, 1);
    for (i = 1; i <= 18; i++) {
        cursor_to_position(cb, 4+i, 1);
        cb_write_number(cb, i, 10);
        cb_write_string(cb, (i < 10 ? "  :?" : " :?"));
    }
    for (i = 19; i<= 22; i++) {
        cursor_to_position(cb, 4+i, 1);
        cb_write_number(cb, 134+i, 10);
        cb_write_string(cb, ":?");
    }
    cursor_to_position(cb, t->input_line, t->input_col);
// 2,1 -> 2, 8 = Time
// 4,1 -> 24, 5 (box) = Switches
// 2,10 -> 10, 30 (box) = Terminal Input
// 2, 32 -> 24, 39 = Sensors
// 26, 1 -> 28, 1 = debug output
}
static inline int parse_command(Command *cmd, circlebuffer_t* cb_input) {
    char c;
    if (cb_empty(cb_input)) {
        cmd->type = INVALID_COMMAND;
        return 0;
    }
    int number, arg, err;
    char swarg;
    cb_read(cb_input, &c);
    switch(c) {
    case 'q':
        if (cb_empty(cb_input)) {
            cmd->type = COMMAND_QUIT;
            return 0;
        }
        break;
    case 't': // tr
        if (cb_empty(cb_input))
            break;
        cb_read(cb_input, &c);
        if (c != 'r')
            break;
        cb_read(cb_input, &c); // the space
        err = cb_read_number(cb_input, &number);
        if (err)
            break;
        err = cb_read_number(cb_input, &arg);
        if (err)
            break;
        // tr <number> <speed>
        cmd->type = COMMAND_TR;
        cmd->arg1 = arg;
        cmd->arg2 = number;
        return 0;
    case 'r': // rv
        if (cb_empty(cb_input))
            break;
        cb_read(cb_input, &c);
        if (c != 'v')
            break;
        cb_read(cb_input, &c); // the space
        err = cb_read_number(cb_input, &number);
        if (err)
            break;
        // rv <number>
        cmd->type = COMMAND_RV;
        cmd->arg1 = number;
        return 0;
    case 's': // sw
        if (cb_empty(cb_input))
            break;
        cb_read(cb_input, &c);
        if (c != 'w')
            break;
        cb_read(cb_input, &c); // the space
        err = cb_read_number(cb_input, &number);
        if (err)
            break;
        err = cb_read(cb_input, &swarg);
        if (err)
            break;
        // sw <number> <direction>
        cmd->type = COMMAND_SW;
        cmd->arg1 = swarg;
        cmd->arg2 = number;
        return 0;
    case 'g': // go
        if (cb_empty(cb_input))
            break;
        cb_read(cb_input, &c);
        if (c != 'o')
            break;
        cmd->type = COMMAND_GO;
        return 0;
    default:
        break;
    }
    while (cb_read(cb_input, &c) == 0);// clear input buffer
    cmd->type = INVALID_COMMAND;
    return 0;
}

void task_terminal_command_parser(int terminaltid){
    int rcv_tid = WhoIs(NAME_UART2_RCV);
    int command_tid = WhoIs(NAME_COMMANDSERVER);
    int err = 0; char c;

    char cb_input_buf[CB_INPUT_BUF_SIZE];
    circlebuffer_t cb_input;
    cb_init(&cb_input, cb_input_buf, CB_INPUT_BUF_SIZE);
    TerminalParser t = {cb_input, {0, 0, 0}};
    TerminalMessage tm = {MESSAGE_TERMINAL, 0, 0};
    ReplyMessage rm = {0, 0};
    
    // main loop
    FOREVER {
        c = Getc(rcv_tid, 2);
        if (c == '\15'){
            err = parse_command(&t.cmd, &t.input);
            tm.rq = TERMINAL_NEWLINE;
            Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
            if (t.cmd.type != INVALID_COMMAND)
                SendCommand(command_tid, t.cmd);
        } else if (c == 8) { //backspace
            cb_backspace(&t.input);
            tm.rq = TERMINAL_BACKSPACE;
            Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
        } else {
            tm.rq = TERMINAL_ECHO;
            tm.arg1 = c;
            Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
            cb_write(&t.input, c);
        }
    }
}

void task_terminal_courier(int servertid){
    int snd_tid = WhoIs(NAME_UART2_SEND);
    TerminalMessage tm = {MESSAGE_TERMINAL, TERMINAL_NOTIFY_COURIER, 0, 0};
    ReplyMessage rm;
    FOREVER {
        Send(servertid, &tm, sizeof(tm), &rm, sizeof(rm));
        Putc(snd_tid, 2, rm.ret);
    }

}

void task_terminal(){
    // concept: server is one of the reciever loop ones
    // Has a cb of characters to put (i guess there's 2 buffers now, which is ??
    // alternate solution: have the 1 buffer, just fix Puts i guess?
    // However, I think this makes sense. The responsibility of what to print for a certain event should be somewhere specific, in this case: here
    RegisterAs(NAME_TERMINAL);
    int mytid = MyTid();
    CreateWithArgument(PRIORITY_HIGH, &task_terminal_command_parser, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_terminal_courier, mytid);

    char cb_terminal_buf[CB_TERMINAL_BUF_SIZE];
    circlebuffer_t cb_terminal;
    cb_init(&cb_terminal, cb_terminal_buf, CB_TERMINAL_BUF_SIZE);
    Terminal t = {cb_terminal, TERMINAL_INPUT_BASE_LINE, TERMINAL_INPUT_BASE_COL, SENSOR_LINE_BASE};
    int tid, err; char c;
    TerminalMessage tm = {0, 0, 0};
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    output_base_terminal(&t);

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));
        if (tm.rq != TERMINAL_NOTIFY_COURIER){
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
        }
        switch (tm.rq) {
        case (TERMINAL_ECHO):
        {
            cb_write(&t.output, tm.arg1);
            break;
        }
        case(TERMINAL_BACKSPACE):
        {
            cb_write_string(&t.output, "\x08 \x08");
            break;
        }
        case(TERMINAL_NEWLINE):
        {
            t.input_line++;
            if (t.input_line >= TERMINAL_INPUT_MAX_LINE) {
                t.input_line = TERMINAL_INPUT_BASE_LINE;
            }
            cursor_to_position(&t.output, t.input_line, TERMINAL_INPUT_BASE_COL);
            cb_write_string(&t.output, "::> \0337");
            for (int i = TERMINAL_INPUT_BASE_COL+4; i < TERMINAL_INPUT_MAX_COL; i++) {
                cb_write(&t.output, ' ');
            }
            cb_write_string(&t.output, "\0338");
            break;
        }
        case(TERMINAL_SENSOR):
        {
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write_string(&t.output, "    ");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write(&t.output, tm.arg1);
            cb_write_number(&t.output, tm.arg2, 10);
            t.sensor_line++;
            if (t.sensor_line > SENSOR_LINE_MAX) {
                t.sensor_line = SENSOR_LINE_BASE;
            }
            cb_write_string(&t.output, "\0338");
            break;
        }
        case(TERMINAL_SWITCH):
        {
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, 4+(tm.arg2 > 18 ? tm.arg2 - 134 : tm.arg2), 5);
            cb_write(&t.output, tm.arg1);
            cb_write_string(&t.output, "\0338");
            break;
        }

        /*
    TERMINAL_TIME,
    TERMINAL_IDLE,
    TERMINAL_MAX,
    TERMINAL_AVG,
    TERMINAL_LST,
    TERMINAL_SNSR,
    TERMINAL_FNSR,
    */

        case(TERMINAL_NOTIFY_COURIER):
        {
            t.notifier = tid;
            break;
        }
        default:
        {
            PANIC("UNHANDLED TERMINAL EXCEPTION");
        }
        }

        if (t.notifier != 0 && !cb_empty(&t.output)){
            err = cb_read(&t.output, (char *) &c);
            rm.ret = c;
            Reply(t.notifier, &rm, sizeof(rm));
            t.notifier = 0;
        }
    }
}
int SendTerminalRequest(int terminaltid, TerminalRequest rq, int arg1, int arg2){
    TerminalMessage tm = {MESSAGE_TERMINAL, rq, arg1, arg2};
    ReplyMessage rm = {0, 0};
    int r = Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
    return ( r >= 0 ? rm.ret : r);
}
