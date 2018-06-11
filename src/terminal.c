#include <kernel.h>
#include <circlebuffer.h>
#include <terminal.h>
#include <name.h>
#include <uart.h>
#include <err.h>
#include <debug.h>

typedef enum cmdtype{
    COMMAND_GO,
    COMMAND_ALL,
    COMMAND_TR,
    COMMAND_RV,
    COMMAND_SW,
    COMMAND_QUIT,
    NO_COMMAND,
    INVALID_COMMAND
} CommandType;

typedef struct command{
    CommandType type;
    int arg1;
    int arg2;
} Command;

typedef struct terminal{
    circlebuffer_t input;
    Command cmd;
} Terminal;

static inline int parse_command(Command *cmd, circlebuffer_t* cb_input){
    char c;
    if (cb_empty(cb_input)) {
        cmd->type = NO_COMMAND;
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
        // 33 = straight
        // 34 = curved
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
    case 'a': // all
        if (cb_empty(cb_input))
            break;
        cb_read(cb_input, &c);
        if (c != 'l')
            break;
        cb_read(cb_input, &c);
        if (c != 'l')
            break;
        cb_read(cb_input, &c); // the space
        err = cb_read(cb_input, &swarg);
        if (err)
            break;
        cmd->type = COMMAND_ALL;
        cmd->arg1 = swarg;
        return 0;
    default:
        break;
    }
    while (cb_read(cb_input, &c) == 0);// clear input buffer
    cmd->type = INVALID_COMMAND;
    return 0;
}

void task_terminal(){
    int rcv_tid = WhoIs(NAME_UART2_RCV);
    int snd_tid = WhoIs(NAME_UART2_SEND);
    int err = 0; char c;

    char cb_input_buf[CB_INPUT_BUF_SIZE];
    circlebuffer_t cb_input;
    cb_init(&cb_input, cb_input_buf, CB_INPUT_BUF_SIZE);
    Terminal t = {cb_input, {0, 0, 0}};
    
    // main loop
    FOREVER {
        c = Getc(rcv_tid, 2);
        Putc(snd_tid, 2, c);
        if (c == '\15'){
            err = parse_command(&t.cmd, &t.input);
            Puts(snd_tid, L(STR_NEWLINE));
            //send t.cmd somewhere to command server
        } else if (c == 8) { //backspace
            cb_backspace(&t.input);
            Puts(snd_tid, L(STR_BACKSPACE));
        } else {
            cb_write(&t.input, c);
        }
    }
}
