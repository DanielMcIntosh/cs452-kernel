/*         E_T:     STK_LIM:     STK_MAX:      DIST_NX:                 RESERVATIONS:
IDLE: %__  E_D:     STK_AVG:     VELO_PR:      SNSR_NX:                  A: 1234567890123456
==---------[ ]--[ ]---------------------------------\                    B: 1234567890123456
==--------[ ] [ ]-----------[ ]----[ ]-----------\   \                   C: 1234567890123456
==--------/  /               \    /               \   |                  D: 1234567890123456
            /                 \||/                 \  |                  E: 1234567890123456
           /                  [  ]                  \ |                  BR:123456789012345678
          |                    ||                   [ ]                      153 154 155 156
          |                    ||                    |                   MR:123456789012345678
          |                    ||                   [ ]                      153 154 155 156
           \                  [  ]                  / |    
            \                / || \                /  |
==------\    \              /      \              /   |
==------[ ]  [ ]-----------[ ]------[ ]---------/    /
==-------[ ]   \-----------[ ]------[ ]-------------/ 
==--------[ ]---------------[ ]----[ ]--------== 
*/

//ABOVE COMMENT IS FOR GETTING LINE, COLUMN VALUES
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
#include <track.h>
#include <terminalcourier.h>

typedef struct terminalparser {
    circlebuffer_t input;
    Command cmd;
} TerminalParser;

typedef struct terminal {
    circlebuffer_t output;
    int input_line;
    int input_col;
    int sensor_line;
    int dbg_col;
    int dbg2_col;
    int pos_col;
    int notifier;
} Terminal;

typedef struct terminalmessage {
    MessageType type;
    TerminalRequest rq;
    int arg1;
    int arg2;
} TerminalMessage;

inline int SendTerminalRequest(int terminaltid, TerminalRequest rq, int arg1, int arg2) {
    TerminalMessage tm = {MESSAGE_TERMINAL, rq, arg1, arg2};
    ReplyMessage rm = {0, 0};
    int r = Send(terminaltid, &tm, sizeof(tm), &rm, sizeof(rm));
    return ( r >= 0 ? rm.ret : r);
}

int terminal_set_reservations(TerminalCourier *tc, Blockage * restrict blockages, int train) {
    int err;
    err = tc_send(tc, TERMINAL_SET_RESRV1, train, blockages->bits_low & 0xFFFFFFFFUL);
    if (err != 0) {
        return err;
    }
    err = tc_send(tc, TERMINAL_SET_RESRV2, train, (blockages->bits_low >> 32) & 0xFFFFFFFFUL);
    if (err != 0) {
        return err;
    }
    err = tc_send(tc, TERMINAL_SET_RESRV3, blockages->bits_high & 0xFFFFFFFFULL, (blockages->bits_high >> 32) & 0xFFFFFFFFULL);
    return err;
}

int terminal_unset_reservations(TerminalCourier *tc, Blockage * restrict blockages) {
    int err;
    err = tc_send(tc, TERMINAL_UNSET_RESRV1, blockages->bits_low & 0xFFFFFFFFUL, (blockages->bits_low >> 32) & 0xFFFFFFFFUL);
    /*
    if (err != 0) {
        return err;
    }
    err = tc_send(tc, TERMINAL_UNSET_RESRV2, train, (blockages->bits_low >> 32) & 0xFFFFFFFFUL);
    if (err != 0) {
        return err;
    }
    err = tc_send(tc, TERMINAL_UNSET_RESRV3, blockages->bits_high & 0xFFFFFFFFULL, (blockages->bits_high >> 32) & 0xFFFFFFFFULL);
    //*/
    return err;
}

static inline void cursor_to_position(struct circlebuffer *cb, int line, int col) {
    cb_write_string(cb, "\033[");
    cb_write_number(cb, line, 10);
    cb_write(cb, ';');
    cb_write_number(cb, col, 10);
    cb_write(cb, 'H');
}

static void output_base_terminal(Terminal *t) {
    const char track_char = GetValue(VALUE_TRACK_NAME);
    //
    //
    const char * topbar = 
    "           E_T:     STK_LIM:     STK_MAX:      DIST_NX:                 RESERVATIONS:\r\n" \
    "IDLE: %__  E_D:     STK_AVG:     VELO_PR:         A_PR:\r\n" ;
    // E_T: 1, 16
    // STK_MAX: 1, 42
    // DIST_NX: 1, 56
    // E_D: 2, 16
    // STK_AVG: 2, 29
    // VELO_PR: 2, 42
    // DIST_PR: 2, 56
    const char * trackA = 
        "==---------[ ]--[ ]---------------------------------\\ \r\n" \
        "==--------[ ] [ ]-----------[ ]----[ ]-----------\\   \\ \r\n" \
        "==--------/  /               \\    /               \\   | \r\n" \
        "            /                 \\||/                 \\  | \r\n" \
        "           /                  [  ]                  \\ | \r\n" \
        "          |                    ||                   [ ] \r\n" \
        "          |                    ||                    | \r\n" \
        "          |                    ||                   [ ] \r\n" \
        "           \\                  [  ]                  / | \r\n" \
        "            \\                / || \\                /  | \r\n" \
        "==------\\    \\              /      \\              /   | \r\n" \
        "==------[ ]  [ ]-----------[ ]------[ ]---------/    / \r\n" \
        "==-------[ ]   \\-----------[ ]------[ ]-------------/  \r\n" \
        "==--------[ ]---------------[ ]----[ ]--------==  \r\n" \
        ;
    const char * trackB = 
        "==---------[ ]--[ ]---------------------------------\\ \r\n" \
        "==--------[ ] [ ]-----------[ ]----[ ]-----------\\   \\ \r\n" \
        "          /  /               \\    /               \\   | \r\n" \
        "         /  /                 \\||/                 \\  | \r\n" \
        "        /  /                  [  ]                  \\ | \r\n" \
        "       |  |                    ||                   [ ] \r\n" \
        "       |  |                    ||                    | \r\n" \
        "       |  |                    ||                   [ ] \r\n" \
        "       |   \\                  [  ]                  / | \r\n" \
        "       |    \\                / || \\                /  | \r\n" \
        "        \\    \\              /      \\              /   | \r\n" \
        "==------[ ]  [ ]-----------[ ]------[ ]---------/    / \r\n" \
        "==-------[ ]   \\-----------[ ]------[ ]-------------/  \r\n" \
        "==--------[ ]---------------[ ]----[ ]--------==  \r\n" \
        ;
    const char * reservables = 
    //we set a temporarily tab stop at 74 just to print this, and another, permanent one at 77
        "\033[2;74H\033H   \033H\010\010\010"
        "A: 1234567890123456\r\n\t"
        "B: 1234567890123456\r\n\t"
        "C: 1234567890123456\r\n\t"
        "D: 1234567890123456\r\n\t"
        "E: 1234567890123456\r\n\t"
        "BR:123456789012345678\r\n\t"
        "    153 154 155 156\r\n\t"
        "MR:123456789012345678\r\n\t"
        "    153 154 155 156\r\n\t"
        //now clear the tab stop
        "\033[g"
        ;
    circlebuffer_t * restrict cb = &t->output;
    ASSERT(cb_write_string(cb, "\033[2J\033[3g\033[H") == 0, "Error outputting base terminal");
    ASSERT(cb_write_string(cb, topbar) == 0, "Error outputting base terminal");
    ASSERT(cb_write_string(cb, track_char == 'A' ? trackA : trackB) == 0, "Error outputting base terminal");
    ASSERT(cb_write_string(cb, reservables) == 0, "Error outputting base terminal");

    cb_write_string(cb, " \033["S(TERMINAL_INPUT_BASE_LINE)";"S(TERMINAL_INPUT_MAX_LINE)"r");
    cursor_to_position(cb, t->input_line, t->input_col);
    cb_write_string(cb, "\r >");
    t->input_col = TERMINAL_INPUT_BASE_COL+3;

    cursor_to_position(cb, TERMINAL_INPUT_MAX_LINE + 1, 1);
    cb_write_string(cb, "TRACK: ");
    cb_write(cb, track_char);
    cb_write_string(cb, "   " STYLED_FLAG_STRING); //"\033["S(TERMINAL_DEBUG_BASE_LINE)";"S(TERMINAL_DEBUG_MAX_LINE)"r");

    cursor_to_position(cb, 1, 29);
    cb_write_number(cb, STACK_SPACE_SIZE/TASK_POOL_SIZE - 4, 16);
    cursor_to_position(cb, t->input_line, t->input_col);
}

static inline int parse_command(Command * restrict cmd, circlebuffer_t * restrict cb_input) {
    char c;
    if (cb_empty(cb_input)) {
        cmd->type = INVALID_COMMAND;
        return 0;
    }
    int err = 0;
    cb_read(cb_input, &c);
    switch(c) {
    case 'q':
    {
        if (cb_empty(cb_input)) {
            cmd->type = COMMAND_QUIT;
            return 0;
        }
        break;
    }
    case 't': // tr
    {
        err = cb_read_match(cb_input, "r ");
        if (err != 0) {
            break;
        }

        int train, speed;
        err = cb_read_number(cb_input, &train);
        if (err)
            break;
        err = cb_read_number(cb_input, &speed);
        if (err)
            break;
        // tr <train> <speed>
        cmd->type = COMMAND_TR;
        cmd->arg1 = speed;
        cmd->arg2 = train;
        return 0;
    }
    case 'r': // rv
    {
        err = cb_read_match(cb_input, "v ");
        if (err != 0) {
            break;
        }

        int train;
        err = cb_read_number(cb_input, &train);
        if (err)
            break;
        // rv <train>
        cmd->type = COMMAND_RV;
        cmd->arg1 = train;
        return 0;
    }
    case 's': // sw
    {
        err = cb_read_match(cb_input, "w ");
        if (err != 0) {
            break;
        }

        int sw;
        char dir;
        err = cb_read_number(cb_input, &sw);
        if (err)
            break;
        err = cb_read(cb_input, &dir);
        if (err)
            break;
        // sw <sw> <dir>
        cmd->type = COMMAND_SW;
        cmd->arg1 = dir;
        cmd->arg2 = sw;
        return 0;
    }
    case 'g': // go
    {
        err = cb_read_match(cb_input, "o");
        if (err != 0) {
            break;
        }

        cmd->type = COMMAND_GO;
        return 0;
    }
    case 'i': // inv
    {
        err = cb_read_match(cb_input, "nv");
        if (err != 0) {
            break;
        }

        cmd->type = COMMAND_INV;
        return 0;
    }
    case 'n': // nav
    {
        err = cb_read_match(cb_input, "av ");
        if (err != 0) {
            break;
        }

        char sensor_char;
        int sensor_num;
        int distance_past;
        int train;

        cb_read(cb_input, &sensor_char); // RADIX
        err = cb_read_number(cb_input, &sensor_num); // SW #
        if (err)
            break;
        err = cb_read_number(cb_input, &distance_past); // distance past sensor
        if (err)
            break;
        err = cb_read_number(cb_input, &train);
        if (err)
            break;        

        // find <sensor> <train>
        cmd->type = COMMAND_ROUTE;
        switch(sensor_char) {
        case ('S'):
        {
            cmd->arg1 = SWITCH_TO_NODE(SWCLAMP(sensor_num));
            break;
        }
        case ('M'):
        {
            cmd->arg1 = MERGE_TO_NODE(SWCLAMP(sensor_num));
            break;
        }
        case ('N'):
        {
            cmd->arg1 = ENTER_TO_NODE(sensor_num - 1);
            break;
        }
        case ('X'):
        {
            cmd->arg1 = EXIT_TO_NODE(sensor_num - 1);
            break;
        }
        case ('A'):
        case ('B'):
        case ('C'):
        case ('D'):
        case ('E'):
        {
            cmd->arg1 = SENSOR_PAIR_TO_SENSOR(sensor_char - 'A', sensor_num - 1);
            break;
        }
        default:
        {
            PANIC("INVALID SENSOR CHAR: %d\r\n", sensor_char)
        }
        }
        cmd->smallarg1 = distance_past;
        cmd->smallarg2 = train;
        return 0;
    }
    case 'm': // move
    {
        err = cb_read_match(cb_input, "ove ");
        if (err != 0) {
            break;
        }

        int distance;
        int train;

        err = cb_read_number(cb_input, &distance); // Distance to move
        if (err)
            break;
        err = cb_read_number(cb_input, &train); // distance past sensor
        if (err)
            break;
        cmd->type = COMMAND_MOVE;
        cmd->arg1 = distance;
        cmd->arg2 = train;
        return 0;
    }
    case 'p': // param
    {
        err = cb_read_match(cb_input, "aram ");
        if (err != 0) {
            break;
        }

        int key;
        char param; // todo string?
        int value; 

        err = cb_read_number(cb_input, &key); // short move array key
        if (err)
            break;
        err = cb_read(cb_input, &param); // param number ('S' = speed, 'D' = time)
        if (err)
            break;
        
        err = cb_read(cb_input, (char *) &err); // space after param
        if (err)
            break;

        err = cb_read_number(cb_input, &value); // param value
        if (err)
            break;
        cmd->type = COMMAND_PARAM;
        cmd->arg1 = value; // arg order is weird because value is the largest potential argument
        cmd->smallarg1 = key;
        cmd->smallarg2 = param;
        return 0;
    }
    case 'a': //add
    {
        err = cb_read_match(cb_input, "dd ");
        if (err != 0) {
            break;
        }

        int train;
        err = cb_read_number(cb_input, &train);
        if (err)
            break;

        char sensor_char;
        int sensor_num;
        cb_read(cb_input, &sensor_char); // RADIX
        err = cb_read_number(cb_input, &sensor_num); // SW #
        if (err)
            break;

        // add <train> <sensor>
        cmd->type = COMMAND_ADD;
        cmd->arg1 = train;
        cmd->arg2 = SENSOR_PAIR_TO_SENSOR(sensor_char - 'A', sensor_num - 1);
        return 0;
    }
    case 'd': // drop
    {
        err = cb_read_match(cb_input, "rop ");
        if (err != 0) {
            break;
        }

        char sensor_char;
        int sensor_num;

        cb_read(cb_input, &sensor_char); // RADIX
        err = cb_read_number(cb_input, &sensor_num); // SW #
        if (err)
            break;

        // drop <node>
        cmd->type = COMMAND_RESERVE;
        switch(sensor_char) {
        case ('S'):
        {
            cmd->arg1 = SWITCH_TO_NODE(SWCLAMP(sensor_num));
            break;
        }
        case ('M'):
        {
            cmd->arg1 = MERGE_TO_NODE(SWCLAMP(sensor_num));
            break;
        }
        case ('N'):
        {
            cmd->arg1 = ENTER_TO_NODE(sensor_num - 1);
            break;
        }
        case ('X'):
        {
            cmd->arg1 = EXIT_TO_NODE(sensor_num - 1);
            break;
        }
        case ('A'):
        case ('B'):
        case ('C'):
        case ('D'):
        case ('E'):
        {
            cmd->arg1 = SENSOR_PAIR_TO_SENSOR(sensor_char - 'A', sensor_num - 1);
            break;
        }
        default:
        {
            PANIC("INVALID SENSOR CHAR: %d\r\n", sensor_char)
        }
        }
        return 0;
    }
    case 'f': // func
    {
        err = cb_read_match(cb_input, "unc ");
        if (err != 0) {
            break;
        }

        int train;
        err = cb_read_number(cb_input, &train);
        if (err)
            break;

        int fn;
        err = cb_read_number(cb_input, &fn); // SW #
        if (err)
            break;

        // add <train> <sensor>
        cmd->type = COMMAND_FUNC;
        cmd->arg1 = train;
        cmd->arg2 = fn;
        return 0;
    }
    default:
    {
        break;
    }
    }

    cmd->type = INVALID_COMMAND;
    return err;
}

void __attribute__((noreturn)) task_terminal_command_parser(int terminaltid){
    int rcv_tid = WhoIs(NAME_UART2_RCV);
    int command_tid = WhoIs(NAME_COMMANDSERVER);
    char c;

    char cb_input_buf[CB_INPUT_BUF_SIZE];
    circlebuffer_t cb_input;
    cb_init(&cb_input, cb_input_buf, CB_INPUT_BUF_SIZE);
    TerminalParser t = {cb_input, {0, 0, {.arg2 = 0}}};
    
    // main loop
    FOREVER {
        c = Getc(rcv_tid, 2);
        if (c == '\15'){
            parse_command(&t.cmd, &t.input);
            cb_flush(&t.input);
            SendTerminalRequest(terminaltid, TERMINAL_NEWLINE, 0, 0);
            if (t.cmd.type != INVALID_COMMAND) {
                SendCommand(command_tid, t.cmd);            
                SendTerminalRequest(terminaltid, TERMINAL_FLAGS_UNSET, STATUS_FLAG_INVALID, 0);
            } else {
                SendTerminalRequest(terminaltid, TERMINAL_FLAGS_SET, STATUS_FLAG_INVALID, 0);
            }
        } else if (c == 8) { //backspace
            if (cb_backspace(&t.input) == 0) {
                SendTerminalRequest(terminaltid, TERMINAL_BACKSPACE, 0, 0);
            }
        } else {
            SendTerminalRequest(terminaltid, TERMINAL_ECHO, c, 0);
            cb_write(&t.input, c);
        }
    }
}

void __attribute__((noreturn)) task_uart2_courier(int servertid) {
    int gettid = WhoIs(NAME_UART2_SEND);
    TerminalMessage tm = {MESSAGE_TERMINAL, TERMINAL_NOTIFY_COURIER, 0, 0};
    ReplyMessage rm;
    FOREVER {
        Send(servertid, &tm, sizeof(tm), &rm, sizeof(rm));
        Putc(gettid, 2, rm.ret);
    }

}

static inline void restylize_string(char * restrict styled_string, unsigned int flags, unsigned int increment, unsigned int start, unsigned int end, char attrib) {
    //append the 'restore cursor' to save a call to cb_write_string
    for (unsigned int i = 1, cur = start; i < end; i <<= 1, cur += increment) {
        if (flags & i) {
            styled_string[cur] = attrib;
        }
    }
}

static inline void print_status(circlebuffer_t * restrict cb, unsigned int status) {
    cb_write_string(cb, "\033[s");
    cursor_to_position(cb, TERMINAL_INPUT_MAX_LINE + 1, 12);

    char str[] = STYLED_FLAG_STRING "\033[u";
    //attribute 1 = Bright
    restylize_string(str, status, 5, 2, STATUS_FLAG_END, '1');
    ASSERT(cb_write_string(cb, str) == 0, "TERMINAL OUTPUT CB FULL");
}

static inline void print_reservations(circlebuffer_t * restrict cb, char *resrv_strs[]) {
    /*
    cb_write_string(cb, "\033[s\033[H\n\t");

    for (int i = 0; i < 2; ++i) {
        unsigned int flags = (reservations >> (i * 16)) & 0xFFFF;
        char str[] = STYLED_RESRV_STRING_1 "\r\n\t";
        restylize_string(str, flags, 6, 3, (0x1U << 16), '1' + train);
        ASSERT(cb_write_string(cb, str) == 0, "TERMINAL OUTPUT CB FULL");
    }
    cb_write_string(cb, "\033[u");
    /*/
    cb_write_string(cb, "\033[s\033[H\n\t");

    for (int i = 0; i < 7; ++i) {
        cb_write_string(cb, resrv_strs[i]);
    }
    cb_write_string(cb, "\033[u");
    //*/
}
static inline void print_reservations2(circlebuffer_t * restrict cb, unsigned int reservations, int train) {
    //*
    cb_write_string(cb, "\033[s\033[4;77H");

    for (int i = 0; i < 2; ++i) {
        unsigned int flags = (reservations >> (i * 16)) & 0xFFFF;
        char str[] = STYLED_RESRV_STRING_1 "\r\n\t";
        restylize_string(str, flags, 6, 3, (0x1U << 16), '1' + train);
        ASSERT(cb_write_string(cb, str) == 0, "TERMINAL OUTPUT CB FULL");
    }
    cb_write_string(cb, "\033[u");
    /*/
    cb_write_string(cb, "\033[s\033[H\n\t");

    for (int i = 0; i < 7; ++i) {
        cb_write_string(cb, resrv_strs[i]);
    }
    cb_write_string(cb, "\033[u");
    //*/
}
static inline void print_reservations3(circlebuffer_t * restrict cb, unsigned long long reservations, int train) {
    cb_write_string(cb, "\033[s\033[6;77H");

    unsigned int flags_snsr = reservations & 0xFFFF;
    char str[] = STYLED_RESRV_STRING_1 "\r\n\t";
    restylize_string(str, flags_snsr, 6, 3, (0x1U << 16), '1' + train);
    ASSERT(cb_write_string(cb, str) == 0, "TERMINAL OUTPUT CB FULL");

    for (int i = 0; i < 2; ++i) {
        unsigned int flags = ((reservations >> (16 + NUM_SWITCHES * i)) & 0x3FFFF);
        unsigned int flags_3way = ((reservations >> (16 + 18 + NUM_SWITCHES * i)) & 0xF);

        char str2[] = STYLED_RESRV_STRING_2 "\r\n\t";
        restylize_string(str2, flags, 6, 3, (0x1U << 18), '1' + train);
        ASSERT(cb_write_string(cb, str2) == 0, "TERMINAL OUTPUT CB FULL");

        char str3[] = STYLED_RESRV_STRING_3 "\r\n\t";
        restylize_string(str3, flags_3way, 9, 3, (0x1U << 4), '1' + train);
        ASSERT(cb_write_string(cb, str3) == 0, "TERMINAL OUTPUT CB FULL");
    }
    cb_write_number(cb, reservations >> 32ULL, 10);
    cb_write(cb, ' ');
    cb_write_number(cb, reservations & 0xFFFFFFFFULL, 10);
    cb_write_string(cb, "\n\r");

    cb_write_string(cb, "\033[u");
}

void __attribute__((noreturn)) task_terminal(int trackstate_tid) {
    const int trackA_switch_positions[][2] = {
        {-1, -1},
        {14, 10},
        {15, 11},
        {16, 12},
        {4, 12},
        {16, 37},
        {15, 29},
        {15, 38},
        {10, 54},
        {8, 54},
        {4, 37},
        {3, 18},
        {3, 13},
        {4, 30},
        {4, 16},
        {14, 15},
        {14, 29},
        {14, 38},
        {16, 30},
        {11, 32},
        {11, 33},
        {7, 33},
        {7, 32}
    };
    // concept: server is one of the reciever loop ones
    // Has a cb of characters to put (now there's 2 buffers)
    // alternate solution: have the 1 buffer, use puts in various places.
    // However, I think this makes sense. The responsibility of what to print for a certain event should be somewhere specific, in this case: here
    RegisterAs(NAME_TERMINAL);
    int mytid = MyTid();
    CreateWithArgument(PRIORITY_HIGH, &task_terminal_command_parser, mytid);
    CreateWithArgument(PRIORITY_NOTIFIER, &task_uart2_courier, mytid);
    CreateWithArgument(PRIORITY_LOW, &task_clockprinter, mytid);
    CreateWithArgument(PRIORITY_HIGHEST, &task_sensor_server, trackstate_tid);
    CreateWithArgument(PRIORITY_LOW, &task_stack_metric_printer, mytid);

    char cb_terminal_buf[CB_TERMINAL_BUF_SIZE];
    circlebuffer_t cb_terminal;
    cb_init(&cb_terminal, cb_terminal_buf, sizeof(cb_terminal_buf));
    Terminal t = {cb_terminal, TERMINAL_INPUT_BASE_LINE, TERMINAL_INPUT_BASE_COL, SENSOR_LINE_BASE, 0, 1, 1, 0};
    int tid, err; char c;
    TerminalMessage tm = {0, 0, 0, 0};
    ReplyMessage rm = {MESSAGE_REPLY, 0};
    unsigned int status = 0;

    char resrv_a[] = STYLED_RESRV_STRING_1 "\r\n\t";
    char resrv_b[] = STYLED_RESRV_STRING_1 "\r\n\t";
    char resrv_c[] = STYLED_RESRV_STRING_1 "\r\n\t";
    char resrv_d[] = STYLED_RESRV_STRING_1 "\r\n\t";
    char resrv_e[] = STYLED_RESRV_STRING_1 "\r\n\t";
    char resrv_br[] = STYLED_RESRV_STRING_2 "\r\n\t" STYLED_RESRV_STRING_3 "\r\n\t";
    char resrv_mr[] = STYLED_RESRV_STRING_2 "\r\n\t" STYLED_RESRV_STRING_3;

    char *resrv_strs[] = { resrv_a, resrv_b, resrv_c, resrv_d, resrv_e, resrv_br, resrv_mr };

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
            if (t.input_col > TERMINAL_INPUT_BASE_COL){
                cb_write_string(&t.output, "\x08 \x08");
                t.input_col--;
            }
            break;
        }
        case(TERMINAL_NEWLINE):
        {
            //wipe the prompt, and move down one line (NOT changing our column!)
            cb_write_string(&t.output, "\r  \n");
            t.input_line++;
            if (t.input_line >= TERMINAL_INPUT_MAX_LINE) {
                t.input_line = TERMINAL_INPUT_MAX_LINE;
            }
            //wipe the previous command
            for (int i = TERMINAL_INPUT_BASE_COL+2; i < TERMINAL_INPUT_MAX_COL; i++) {
                cb_write(&t.output, ' ');
            }
            t.input_col = TERMINAL_INPUT_BASE_COL + 2;
            //write the prompt
            cb_write_string(&t.output, "\r> ");
            break;
        }
        case(TERMINAL_SENSOR):
        {
            int sensor = tm.arg1;
            int sensor_radix = SENSOR_GET_RADIX(sensor);
            int sensor_num = SENSOR_GET_NUM(sensor) + 1; //change from 0 based to 1 based sensor values
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write_string(&t.output, "    ");
            cursor_to_position(&t.output, t.sensor_line, SENSOR_COL_BASE);
            cb_write(&t.output, 'A' + sensor_radix);
            if (sensor_num < 10)
                cb_write(&t.output, '0');
            cb_write_number(&t.output, sensor_num, 10);
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
            cursor_to_position(&t.output, trackA_switch_positions[SWCLAMP(tm.arg2)][0], trackA_switch_positions[SWCLAMP(tm.arg2)][1]);
            cb_write(&t.output, tm.arg1);
            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_TIME):
        {
            unsigned int time_hundred_millis = (unsigned int)tm.arg1, idle = (unsigned int)tm.arg2; 
            
            cb_write_string(&t.output, "\0337\033[H");
            unsigned int m = time_hundred_millis % 10;
            time_hundred_millis /= 10;
            unsigned int ss = time_hundred_millis % 60;
            time_hundred_millis /= 60;
            unsigned int mm = time_hundred_millis;

            cb_write_fixed_size_number(&t.output, mm, 10, 10);
            cb_write(&t.output, ':');
            cb_write_fixed_size_number(&t.output, ss, 10, 10);
            cb_write(&t.output, ':');
            cb_write_fixed_size_number(&t.output, m, 10, 1);

            cb_write_string(&t.output, "\n");
            cb_write_fixed_size_number(&t.output, idle, 10, 10);

            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_STACK_METRICS):
        {
            int avg_stack = tm.arg1, max_stack = tm.arg2;
            cb_write_string(&t.output, "\0337");

            cursor_to_position(&t.output, 2, 29);
            cb_write_number(&t.output, avg_stack, 16);
            cb_write_string(&t.output, "  ");

            cursor_to_position(&t.output, 1, 42);
            cb_write_number(&t.output, max_stack, 16);
            cb_write_string(&t.output, "  ");

            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_SENSOR_PREDICT):
        {
            int last_error_time = tm.arg1;
            int last_error_dist = tm.arg2;
            cb_write_string(&t.output, "\0337");

            cursor_to_position(&t.output, 1, 16);
            cb_write_number(&t.output, last_error_time, 10);
            cb_write_string(&t.output, "   ");

            cursor_to_position(&t.output, 2, 16);
            cb_write_number(&t.output, last_error_dist, 10);
            cb_write_string(&t.output, "   ");

            cb_write_string(&t.output, "\0338");

            break;
        }
        case (TERMINAL_VELOCITY_DEBUG):
        {
            int current_predicted_velocity = tm.arg1;
            int prev_dist = tm.arg2;
            cb_write_string(&t.output, "\0337");

            cursor_to_position(&t.output, 2, 42);
            cb_write_number(&t.output, current_predicted_velocity, 10);
            cb_write_string(&t.output, "   ");

            cursor_to_position(&t.output, 2, 56);
            cb_write_number(&t.output, prev_dist, 10);
            cb_write_string(&t.output, "   ");

            cb_write_string(&t.output, "\0338");

            break;
        }
        case (TERMINAL_ROUTE_DBG):
        {
            int sw = tm.arg1;
            int action = tm.arg2;
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, TERMINAL_INPUT_MAX_LINE+2, t.dbg_col);
            t.dbg_col += 4;
            if (t.dbg_col > 50) t.dbg_col = 0;
            cb_write_number(&t.output, sw, 10);
            cb_write_string(&t.output, (action == ACTION_STRAIGHT ? "S" : (action == ACTION_CURVED ? "C" : "R")));
            cb_write_string(&t.output, " ");
            cb_write_string(&t.output, "\0338");
            break;
        }
        case (TERMINAL_ROUTE_DBG2):
        {
            int n = tm.arg1;
            int n2 = tm.arg2;
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, TERMINAL_DEBUG_BASE_LINE, t.dbg2_col);
            t.dbg2_col += cb_write_number(&t.output, n, 10);
            cb_write_string(&t.output, ":");
            t.dbg2_col += cb_write_number(&t.output, n2, 10);
            cb_write_string(&t.output, " ");
            t.dbg2_col += 2;
            if (t.dbg2_col > 97) {
                //cb_write_string(&t.output, "\n");
                t.dbg2_col = 1;
            }
            cb_write_string(&t.output, "\0338");
            break;
        }
        case(TERMINAL_FLAGS_SET):
        {
            unsigned int flags = (unsigned int)tm.arg1;
            if (~status & flags)
            {
                status |= flags;

                print_status(&t.output, status);
            }
            break;
        }
        case(TERMINAL_FLAGS_UNSET):
        {
            unsigned int flags = (unsigned int)tm.arg1;
            if (status & flags) {
                status &= ~flags;

                print_status(&t.output, status);
            }
            break;
        }
        case(TERMINAL_SET_RESRV1):
        {
            int active_train = tm.arg1;
            unsigned int flags_a = ((unsigned int)tm.arg2 >> 0 ) & 0xFFFF;
            unsigned int flags_b = ((unsigned int)tm.arg2 >> 16) & 0xFFFF;

            restylize_string(resrv_a, flags_a, 6, 3, (0x1U << 16), '1' + active_train);
            restylize_string(resrv_b, flags_b, 6, 3, (0x1U << 16), '1' + active_train);

            //terminal_update_reservations will call TERMINAL_SET_RESRV2, so don't bother printing yet
            //print_reservations(&t.output, resrv_strs, active_train);
            break;
        }
        case(TERMINAL_SET_RESRV2):
        {
            int active_train = tm.arg1;
            unsigned int flags_c = ((unsigned int)tm.arg2 >> 0 ) & 0xFFFF;
            unsigned int flags_d = ((unsigned int)tm.arg2 >> 16) & 0xFFFF;

            restylize_string(resrv_c, flags_c, 6, 3, (0x1U << 16), '1' + active_train);
            restylize_string(resrv_d, flags_d, 6, 3, (0x1U << 16), '1' + active_train);

            print_reservations(&t.output, resrv_strs);
            break;
        }
        case(TERMINAL_SET_RESRV3):
        {
            int active_train = 4;
            unsigned long long flags = (unsigned int)tm.arg1 | (((unsigned long long)tm.arg2) << 32ULL);
            print_reservations3(&t.output, flags, active_train);
            break;
        }
        case(TERMINAL_UNSET_RESRV1):
        {
            unsigned int flags_a = ((unsigned int)tm.arg1 >> 0 ) & 0xFFFF;
            unsigned int flags_b = ((unsigned int)tm.arg1 >> 16) & 0xFFFF;
            unsigned int flags_c = ((unsigned int)tm.arg2 >> 0 ) & 0xFFFF;
            unsigned int flags_d = ((unsigned int)tm.arg2 >> 16) & 0xFFFF;

            restylize_string(resrv_a, flags_a, 6, 3, (0x1U << 16), '7');
            restylize_string(resrv_b, flags_b, 6, 3, (0x1U << 16), '7');
            restylize_string(resrv_c, flags_c, 6, 3, (0x1U << 16), '7');
            restylize_string(resrv_d, flags_d, 6, 3, (0x1U << 16), '7');

            print_reservations(&t.output, resrv_strs);
            break;
        }
        case (TERMINAL_POS_DBG):
        {
            int node = tm.arg1;
            int dist = tm.arg2;
            track_node * tn = &track[node];
            const char *n = tn->name;
            cb_write_string(&t.output, "\0337");
            cursor_to_position(&t.output, TERMINAL_DEBUG_MAX_LINE+1, t.pos_col);
            while (*n != '\0'){
                t.pos_col++;
                cb_write(&t.output, *n);
                n++;
            }
            cb_write(&t.output,'+');
            t.pos_col++;
            t.pos_col += cb_write_number(&t.output, dist, 10);
            if (t.pos_col > 97) {
                t.pos_col = 1;
            }
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
