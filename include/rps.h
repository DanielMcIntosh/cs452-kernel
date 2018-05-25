#ifndef RPS_H
#define RPS_H

#define RPS_NAME "RPS"

typedef enum {
    ROCK,
    PAPER,
    SCISSORS,
    QUIT
} RPS;

typedef enum {
    WIN,
    LOSE,
    TIE,
    OPPONENT_FOUND,
    OPPONENT_QUIT
} RPSStatus;

int Signup(int rps_tid);
int Play(int rps_tid, RPS move, RPSStatus* reply);
int Quit(int rps_tid);

void task_rps();
void task_rps_client();

#endif
