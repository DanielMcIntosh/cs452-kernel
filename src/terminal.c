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
#include <clock.h>
#include <sensor.h>

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
    cb_write_string(cb, "\033[");
    cb_write_number(cb, line, 10);
    cb_write(cb, ';');
    cb_write_number(cb, col, 10);
    cb_write(cb, 'H');
}

static void output_base_terminal(Terminal *t) {
    circlebuffer_t *cb = &t->output;
    cb_write_string(cb, "\033[2J\033[3g\033[H\n");
    cb_write_string(cb, "IDLE: %\r\n");
    cb_write_string(cb, "NXT:\r\n");
    cb_write_string(cb, "ERR:\r\n");
    int i;
    cursor_to_position(cb, t->input_line, t->input_col);
    cb_write_string(cb, "\033H> ");
    t->input_col+= 2;

    cursor_to_position(cb, 27, 1);
    cb_write_string(cb, "STK_LIM: ");
    cb_write_number(cb, STACK_SPACE_SIZE/TASK_POOL_SIZE - 4, 16);
    cb_write_string(cb, "\r\nSTK_AVG: \r\n");
    cb_write_string(cb, "STK_MAX: \r\n");

    cursor_to_position(cb, 5, 1);
    for (i = 1; i <= 18; i++) {
        cb_write_number(cb, i, 10);
        cb_write_string(cb, (i < 10 ? "  :?\r\n" : " :?\r\n"));
    }
    for (i = 19; i<= 22; i++) {
        cb_write_number(cb, 134+i, 10);
        cb_write_string(cb, ":?\r\n");
    }
    cursor_to_position(cb, t->input_line, t->input_col);
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
    TerminalMessage tm = {MESSAGE_TERMINAL, 0, 0, 0};
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
            if (cb_backspace(&t.input) == 0) {
                tm.rq = TERMINAL_BACKSPACE;
                Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
            }
        } else {
            tm.rq = TERMINAL_ECHO;
            tm.arg1 = c;
            Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
            cb_write(&t.input, c);
        }
    }
}

void task_terminal_courier(int servertid) {
    int gettid = WhoIs(NAME_UART2_SEND);
    TerminalMessage tm = {MESSAGE_TERMINAL, TERMINAL_NOTIFY_COURIER, 0, 0};
    ReplyMessage rm;
    FOREVER {
        Send(servertid, &tm, sizeof(tm), &rm, sizeof(rm));
        Putc(gettid, 2, rm.ret);
    }

}

void task_terminal() {
    // concept: server is one of the reciever loop ones
    // Has a cb of characters to put (now there's 2 buffers)
    // alternate solution: have the 1 buffer, use puts in various places.
    // However, I think this makes sense. The responsibility of what to print for a certain event should be somewhere specific, in this case: here
    RegisterAs(NAME_TERMINAL);
    int mytid = MyTid();
    CreateWithArgument(PRIORITY_HIGH, &task_terminal_command_parser, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_terminal_courier, mytid);
    CreateWithArgument(PRIORITY_LOW, &task_clockprinter, mytid);
    Create(PRIORITY_HIGH, &task_sensor_server);
    CreateWithArgument(PRIORITY_LOW, &task_stack_metric_printer, mytid);

    char cb_terminal_buf[CB_TERMINAL_BUF_SIZE];
    circlebuffer_t cb_terminal;
    cb_init(&cb_terminal, cb_terminal_buf, CB_TERMINAL_BUF_SIZE);
    Terminal t = {cb_terminal, TERMINAL_INPUT_BASE_LINE, TERMINAL_INPUT_BASE_COL, SENSOR_LINE_BASE, 0};
    int tid, err; char c;
    TerminalMessage tm = {0, 0, 0, 0};
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    output_base_terminal(&t);

    FOREVER{
        Receive(&tid, &tm, sizeof(tm));
        if (tm.rq != TERMINAL_NOTIFY_COURIER) {
            rm.ret = 0;
            Reply(tid, &rm, sizeof(rm));
        }
        switch (tm.rq) {
        case (TERMINAL_ECHO):
        {
            cb_write(&t.output, tm.arg1);
            t.input_col++;
            break;
        }
        case(TERMINAL_BACKSPACE):
        {
            if (t.input_col > TERMINAL_INPUT_BASE_COL)
                cb_write_string(&t.output, "\x08 \x08");
            break;
        }
        case(TERMINAL_NEWLINE):
        {
            //wipe the prompt, and move down one line (NOT changing our column!)
            cb_write_string(&t.output, "\r\t  \n");
            t.input_line++;
            if (t.input_line >= TERMINAL_INPUT_MAX_LINE) {
                t.input_line = TERMINAL_INPUT_BASE_LINE;
                cursor_to_position(&t.output, t.input_line, TERMINAL_INPUT_BASE_COL+2);
            }
            //wipe the previous command
            for (int i = TERMINAL_INPUT_BASE_COL+2; i < TERMINAL_INPUT_MAX_COL; i++) {
                cb_write(&t.output, ' ');
            }
            //write the prompt
            cb_write_string(&t.output, "\r\t> ");
            break;
        }
        case(TERMINAL_SENSOR):
        {
            char radix = tm.arg1 >> 16;
            int snsr = tm.arg1 & 0xFF; // unpack arg1;
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write_string(&t.output, "    ");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write(&t.output, 'A' + radix);
            if (snsr < 10)
                cb_write(&t.output, '0');
            cb_write_number(&t.output, snsr, 10);
            cb_write_string(&t.output, " (");
            cb_write_number(&t.output, tm.arg2, 10);
            cb_write(&t.output, ')');
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
        case (TERMINAL_TIME):
        {
            int time_hundred_millis = tm.arg1, idle = tm.arg2; 
            
            cb_write_string(&t.output, "\0337\033[H");
            int m = time_hundred_millis % 10;
            time_hundred_millis /= 10;
            int ss = time_hundred_millis % 60;
            time_hundred_millis /= 60;
            int mm = time_hundred_millis;
            if (mm < 10)
                cb_write(&t.output, '0');
            cb_write_number(&t.output, mm, 10);
            cb_write(&t.output, ':');
            if (ss < 10)
                cb_write(&t.output, '0');
            cb_write_number(&t.output, ss, 10);
            cb_write(&t.output, ':');
            cb_write_number(&t.output, m, 10);

            cb_write_string(&t.output, "\n");
            cb_write_number(&t.output, idle, 10);

            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_STACK_METRICS):
        {
            int avg_stack = tm.arg1, max_stack = tm.arg2;
            cb_write_string(&t.output, "\0337");

            cursor_to_position(&t.output, 28, 10);
            cb_write_number(&t.output, avg_stack, 16);
            cb_write_string(&t.output, "   ");

            cursor_to_position(&t.output, 29, 10);
            cb_write_number(&t.output, max_stack, 16);
            cb_write_string(&t.output, "   ");

            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_SENSOR_PREDICT):
        {
            int next_sensor_predicted_time = tm.arg1;
            int last_difference = tm.arg2;
            cb_write_string(&t.output, "\0337");

            cursor_to_position(&t.output, 3, 5);
            cb_write_number(&t.output, next_sensor_predicted_time, 10);
            cb_write_string(&t.output, "   ");

            cursor_to_position(&t.output, 4, 5);
            cb_write_number(&t.output, last_difference, 10);
            cb_write_string(&t.output, "   ");

            cb_write_string(&t.output, "\0338");

            break;
        }
        case(TERMINAL_NOTIFY_COURIER):
        {
            t.notifier = tid;
            break;
        }
        default:
        {
            PANIC("UNHANDLED TERMINAL EXCEPTION: %d", tm.rq);
        }
        }
        
        if (t.notifier != 0 && !cb_empty(&t.output)) {
            err = cb_read(&t.output, (char *) &c);
            ASSERT(err == 0, "Error reading from circular buffer");
            rm.ret = c;
            Reply(t.notifier, &rm, sizeof(rm));
            t.notifier = 0;
        }
    }
}

int SendTerminalRequest(int terminaltid, TerminalRequest rq, int arg1, int arg2) {
    TerminalMessage tm = {MESSAGE_TERMINAL, rq, arg1, arg2};
    ReplyMessage rm = {0, 0};
    int r = Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
    return ( r >= 0 ? rm.ret : r);
}
