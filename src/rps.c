#include <rps.h>
#include <message.h>
#include <syscall.h>
#include <kernel.h>
#include <name.h>
#include <err.h>
#include <debug.h>
#include <tasks.h>
#include <clock.h>

typedef struct {
    MessageType type;
    RPS move;
} RPSMessage;

int Signup(int rps_tid){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_SIGNUP;
    int size = Send(rps_tid, &msg, sizeof(msg), &msg, sizeof(msg));
    return (size == sizeof(msg)) ? 0 : size;
}
int Play(int rps_tid, RPS move, RPSStatus* reply){
    RPSMessage msg;
    ReplyMessage rply;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = move;
    int size = Send(rps_tid, &msg, sizeof(msg), &rply, sizeof(rply));
    *reply = rply.ret;
    return (size == sizeof(rply)) ? 0 : size;
}
int Quit(int rps_tid){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = QUIT;
    int size = Send(rps_tid, &msg, sizeof(msg), &msg, sizeof(msg));
    return (size == sizeof(msg)) ? 0 : size;
}

int wins(RPS a, RPS b){
    if (a == b) return TIE;
    if ((b + 1) % 3 == a) return WIN;
    return LOSE;
}

typedef struct {
    int games[TASK_POOL_SIZE];
    int plays[TASK_POOL_SIZE];
    int unpaired; // this is single threaded - no point in a queue
} RPSServer;

void task_rps(){
    LOG("RPS Server Start");
    RPSServer rps;
    for (int i = 0; i < TASK_POOL_SIZE; i++){
        rps.games[i] = -1;
        rps.plays[i] = -1;
    }
    rps.unpaired = -1;

    LOG("RPS Server Registration");
    RegisterAs(RPS_NAME);
    int tid, size, err, opp, play;
    RPSMessage msg;
    ReplyMessage rply;
    rply.type = MESSAGE_REPLY;
    LOG("RPS Server Init");
    FOREVER {
        size = Receive(&tid, &msg, sizeof(msg));
        ASSERT(size == sizeof(msg), "Error recieving complete message",);
        LOGF("RPS Server recieved: %d, %d\r\n", msg.type, msg.move);
        tid &= TASK_BASE_TID_MASK;
        switch (msg.type) {
        case MESSAGE_RPS_SIGNUP:
            if (rps.unpaired == -1){
                rps.unpaired = tid;
            } else {
                rps.games[tid] = rps.unpaired;
                rps.games[rps.unpaired] = tid;
                rply.ret = OPPONENT_FOUND;
                LOGF("RPS Server replying to: %d\r\n", tid);
                err = Reply(tid, &rply, sizeof(rply));
                LOGF("RPS Server replying to: %d\r\n", rps.unpaired);
                err = Reply(rps.unpaired, &rply, sizeof(rply));
                rps.unpaired = -1;
            }
            break;
        case MESSAGE_RPS_PLAY:
            opp = rps.games[tid];
            if (opp == -1) {
                rply.ret = ERR_NO_OPPONENT;
                err = Reply(opp, &rply, sizeof(rply));
                ASSERT(err == 0, "Error replying to message",);
                break;
            }
            play = rps.plays[opp];
            if (play != -1) {
                // opponent has already made a move
                if (play == QUIT){
                    rply.ret = OPPONENT_QUIT;
                    rps.plays[opp] = -1;
                    rps.games[tid] = -1;
                    rps.games[opp] = tid;
                    LOGF("RPS Server replying to: %d\r\n", tid);
                    err = Reply(tid, &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                } else {
                    rply.ret = wins(play, msg.move);
                    LOGF("RPS Server replying to: %d\r\n", opp);
                    err = Reply(opp, &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                    rply.ret = wins(msg.move, play);
                    LOGF("RPS Server replying to: %d\r\n", tid);
                    err = Reply(tid, &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                    rps.plays[opp] = -1;
                }
            } else {
                rps.plays[tid] = msg.move;
            }
            break;
        default:
            rply.ret = ERR_INVALID_ARGUMENT;
            Reply(tid, &rply, sizeof(rply));
        }
    }
    Exit();
}

void task_rps_client(){
    int rps_tid = WhoIs(RPS_NAME);
    int mytid = MyTid();
    int err = Signup(rps_tid);
    RPSStatus reply;
    if (err){
        bwprintf(COM2, "%d: Error with RPS Signup: %d\r\n", mytid, err);
        Exit();
    }
    for (int i = 0; i < 10; i++){
        RPS move = clk4->value_low % 3; //randomization using clock time
        err = Play(rps_tid, move, &reply);
        if (err){
            bwprintf(COM2, "%d: Error playing RPS Move: %d\r\n", mytid,  err);
            break;
        } else if (reply == OPPONENT_QUIT) {
            bwprintf(COM2, "%d: Opponent Quit\r\n", mytid);
            break;

        }
        bwprintf(COM2, "%d: I played: %s, result: %s\r\n", mytid,  
            (move == ROCK ? "ROCK" : (move == PAPER ? "PAPER" : "SCISSORS")),
            (reply == WIN ? "WIN" : (reply == LOSE ? "LOSE" : "TIE")));
        bwgetc(COM2);
    }
    Quit(rps_tid);
    Exit();
}
