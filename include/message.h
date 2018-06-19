#ifndef MESSAGE_H
#define MESSAGE_H

typedef enum {
    MESSAGE_REPLY,
    MESSAGE_WHOIS,
    MESSAGE_REGAS,
    MESSAGE_NAME_LOOKUP,
    MESSAGE_RPS_SIGNUP,
    MESSAGE_RPS_PLAY,
    MESSAGE_RPS_QUIT,
    MESSAGE_CLOCK,
    MESSAGE_TT,
    MESSAGE_UART,
    MESSAGE_COMMAND,
    MESSAGE_SENSOR,
    MESSAGE_SENSOR_COURIER,
    MESSAGE_TERMINAL,
    MESSAGE_TERMINAL_COURIER,
    MESSAGE_TRACK_STATE,
    NUM_MESSAGE_TYPES
} MessageType;

typedef struct {
    MessageType type;
    int ret;
} ReplyMessage;

#endif
