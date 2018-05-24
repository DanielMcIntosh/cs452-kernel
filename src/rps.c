// big TODO
#include <rps.h>
#include <message.h>
#include <syscall.h>
#include <kernel.h>
#include <name.h>
#include <err.h>
#include <debug.h>

typedef struct {
    MessageType type;
    RPS move;
} RPSMessage;

int Signup(int rps_tid){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_SIGNUP;
    return Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&msg, sizeof(msg));
}
int Play(int rps_tid, RPS move){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = move;
    return Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&msg, sizeof(msg));
}
int Quit(int rps_tid){
    RPSMessage msg;
    msg.type = MESSAGE_RPS_PLAY;
    msg.move = QUIT;
    return Send(rps_tid, (void*)&msg, sizeof(msg), (void*)&msg, sizeof(msg));
}

int wins(RPS a, RPS b){ // TODO: check if this is even correct
    if (a == b) return TIE;
    if ((a - b) % 3 == 1) return WIN;
    return LOSE;
}

typedef struct {
    int games[100]; // TODO MAX_TIDs
    int plays[100];
    int unpaired; // this is single threaded - no point in a queue
} RPSServer;

void task_rps(){
    RPSServer rps;
    for (int i = 0; i < 100; i++){
        rps.games[i] = -1;
        rps.plays[i] = -1;
    }
    rps.unpaired = -1;

    RegisterAs(RPS_NAME);
    int tid, err, opp, play;
    RPSMessage msg;
    ReplyMessage rply;
    rply.type = MESSAGE_REPLY;
    FOREVER {
        err = Receive(&tid, (void *) &msg, sizeof(msg));
        ASSERT(err == 0, "Error recieving message",);
        switch (msg.type) {
        case MESSAGE_RPS_SIGNUP:
            if (rps.unpaired == -1){
                rps.unpaired = tid;
            } else {
                rps.games[tid] = rps.unpaired;
                rps.games[rps.unpaired] = tid;
                rply.ret = OPPONENT_FOUND;
                err = Reply(tid, (void *) &rply, sizeof(rply));
                ASSERT(err == 0, "Error recieving message",);
                err = Reply(rps.unpaired, (void *) &rply, sizeof(rply));
                ASSERT(err == 0, "Error recieving message",);
                rps.unpaired = -1;
            }
            break;
        case MESSAGE_RPS_PLAY:
            opp = rps.games[tid];
            if (opp == -1) {
                rply.ret = ERR_NO_OPPONENT;
            }
            play = rps.plays[opp];
            if (play != -1) {
                // opponent has already made a move
                if (play == QUIT){
                    rply.ret = OPPONENT_QUIT;
                    rps.plays[opp] = -1;
                    rps.games[tid] = -1;
                    rps.games[opp] = tid;
                } else {
                    rply.ret = wins(play, msg.move);
                    err = Reply(opp, (void*) &rply, sizeof(rply));
                    ASSERT(err == 0, "Error recieving message",);
                    rply.ret = wins(msg.move, play);
                    err = Reply(tid, (void *) &rply, sizeof(rply));
                    ASSERT(err == 0, "Error recieving message",);// TODO swich string to replying
                    rps.plays[opp] = -1;
                }
            } else {
                rps.plays[tid] = play;
            }
            break;
        default:
            rply.ret = ERR_INVALID_ARGUMENT;
            Reply(tid, (void *) &rply, sizeof(rply));
        }
    }
}
