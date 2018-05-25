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
    return Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&msg, sizeof(msg));
}
int Play(int rps_tid, RPS move, RPSStatus* reply){
    RPSMessage msg;
    ReplyMessage rply;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = move;
    int err = Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&rply, sizeof(rply));
    *reply = rply.ret;
    return err;
}
int Quit(int rps_tid){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = QUIT;
    return Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&msg, sizeof(msg));
}

int wins(RPS a, RPS b){
    if (a == b) return TIE;
    if ((a - b) % 3 == 1) return WIN;
    return LOSE;
}

typedef struct {
    int games[TASK_POOL_SIZE];
    int plays[TASK_POOL_SIZE];
    int unpaired; // this is single threaded - no point in a queue
} RPSServer;

void task_rps(){
    #if DEBUG
    bwputstr(COM2, "RPS Server Start");
    #endif
    RPSServer rps;
    for (int i = 0; i < TASK_POOL_SIZE; i++){
        rps.games[i] = -1;
        rps.plays[i] = -1;
    }
    rps.unpaired = -1;

    #if DEBUG
    bwputstr(COM2, "RPS Server Registration");
    #endif
    RegisterAs(RPS_NAME);
    int tid, err, opp, play;
    RPSMessage msg;
    ReplyMessage rply;
    rply.type = MESSAGE_REPLY;
    #if DEBUG
    bwputstr(COM2, "RPS Server Init");
    #endif
    FOREVER {
        err = Receive(&tid, (void *) &msg, sizeof(msg));
        ASSERT(err == 0, "Error recieving message",);
        #if DEBUG
        bwprintf(COM2, "RPS Server recieved: %d, %d\r\n", msg.type, msg.move);
        #endif
        tid &= TASK_BASE_TID_MASK;
        switch (msg.type) {
        case MESSAGE_RPS_SIGNUP:
            if (rps.unpaired == -1){
                rps.unpaired = tid;
            } else {
                rps.games[tid] = rps.unpaired;
                rps.games[rps.unpaired] = tid;
                rply.ret = OPPONENT_FOUND;
                #if DEBUG
                bwprintf(COM2, "RPS Server replying to: %d\r\n", tid);
                #endif
                err = Reply(tid, (void *) &rply, sizeof(rply));
                #if DEBUG
                bwprintf(COM2, "RPS Server replying to: %d\r\n", rps.unpaired);
                #endif
                err = Reply(rps.unpaired, (void *) &rply, sizeof(rply));
                rps.unpaired = -1;
            }
            break;
        case MESSAGE_RPS_PLAY:
            opp = rps.games[tid];
            if (opp == -1) {
                rply.ret = ERR_NO_OPPONENT;
                err = Reply(opp, (void*) &rply, sizeof(rply));
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
                    #if DEBUG
                    bwprintf(COM2, "RPS Server replying to: %d\r\n", tid);
                    #endif
                    err = Reply(tid, (void*) &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                } else {
                    rply.ret = wins(play, msg.move);
                    #if DEBUG
                    bwprintf(COM2, "RPS Server replying to: %d\r\n", opp);
                    #endif
                    err = Reply(opp, (void*) &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                    rply.ret = wins(msg.move, play);
                    #if DEBUG
                    bwprintf(COM2, "RPS Server replying to: %d\r\n", tid);
                    #endif
                    err = Reply(tid, (void *) &rply, sizeof(rply));
                    ASSERT(err == 0, "Error replying to message",);
                    rps.plays[opp] = -1;
                }
            } else {
                rps.plays[tid] = msg.move;
            }
            break;
        default:
            rply.ret = ERR_INVALID_ARGUMENT;
            Reply(tid, (void *) &rply, sizeof(rply));
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
