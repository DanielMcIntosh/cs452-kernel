#include <name.h>
#include <kernel.h>
#include <syscall.h>
#include <message.h>
#include <debug.h>
#include <err.h>
#include <util.h>
#include <bwio.h>
#include <hashtable.h>

typedef struct nameserver {
    Hashtable ht;
} NameServer;

typedef struct namemessage {
    MessageType id;
    char name [MAXNAMESIZE];
    int tid;
} NameMessage;

int TID_NS = 0;

int legal_name(char * n){
    int len = 0;
    while (*n++ != NULL)
        len++;
    return len <= 8;
}

void task_nameserver(){
    LOG("NameServer init\r\n");
    NameServer ns;
    ht_init(&ns.ht);
    TID_NS = MyTid();
    int tid, size;
    NameMessage msg;
    LOGF("NameServer (%d) begin loop", TID_NS);
    FOREVER { 
        size = Receive(&tid, &msg, sizeof(msg));
        if (size != sizeof(msg)){
            LOGF("NameServer error: %d\r\n ", size);
            LOGF("Message: %d %s %d\r\n", msg.id, msg.name, msg.tid);
            msg.tid = ERR_MSG_TRUNCATED;
        } else if (!legal_name(msg.name)){
            msg.tid = ERR_INVALID_ARGUMENT;
        } else {
            switch (msg.id) {
            case MESSAGE_WHOIS:
                msg.tid = ht_lookup(&ns.ht, msg.name);
                LOGF("NameServer found %s as %d\r\n ", msg.name, msg.tid);
                break;
            case MESSAGE_REGAS:
                LOGF("NameServer registering %s as %d\r\n ", msg.name, tid);
                msg.tid = ht_insert(&ns.ht, msg.name, tid);
                LOGF("NameServer registered %s as %d\r\n ", msg.name, tid);
                break;
            case MESSAGE_NAME_LOOKUP:
                ht_rev_lookup(&ns.ht, msg.tid, msg.name);
            default:
                LOGF("NameServer received invalid argument %d\r\n ", msg.id);
                msg.tid = ERR_INVALID_ARGUMENT;
                break;
            }
        }
        Reply(tid, &msg, sizeof(msg));
    }
}

int RegisterAs(char * name) {
    NameMessage msg;
    msg.id = MESSAGE_REGAS;
    memcpy(msg.name, name, MAXNAMESIZE); 
    Send(TID_NS, &msg, sizeof(msg), &msg, sizeof(msg));
    return msg.tid;
}

int WhoIs(char * name){
    NameMessage msg;
    msg.id = MESSAGE_WHOIS;
    memcpy(msg.name, name, MAXNAMESIZE);
    Send(TID_NS, &msg, sizeof(msg), &msg, sizeof(msg));
    return msg.tid;
}

char *NameLookup(int tid, char *result_buf) {
    NameMessage msg;
    msg.id = MESSAGE_NAME_LOOKUP;
    msg.tid = tid;
    Send(TID_NS, &msg, sizeof(msg), &msg, sizeof(msg));
    memcpy(result_buf, msg.name, MAXNAMESIZE);
    return result_buf;
}
