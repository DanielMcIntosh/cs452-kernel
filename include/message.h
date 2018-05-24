#ifndef MESSAGE_H
#define MESSAGE_H

typedef enum {
    MESSAGE_REPLY,
    MESSAGE_WHOIS,
    MESSAGE_REGAS,
    MESSAGE_RPS_SIGNUP,
    MESSAGE_RPS_PLAY,
    MESSAGE_RPS_QUIT
} MessageType;

typedef struct {
    MessageType type;
    int ret;
} ReplyMessage;

#endif
