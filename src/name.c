// big TODO
#include <name.h>
#include <kernel.h>
#include <syscall.h>
#include <message.h>
#include <debug.h>
#include <err.h>
#include <util.h>
#include <bwio.h>

typedef struct nameserver {
    int names[NUM_NAMES]; //the worlds dankest hashtable
} NameServer;

typedef struct namemessage {
    MessageType id;
    char name [MAXNAMESIZE];
    int tid;
} NameMessage;

int hash(char * x) { // TODO temporary
    int h = 0;

    return h;
}

int TID_NS = 0;

void task_nameserver(){
    #if DEBUG
    bwputstr(COM2, "NameServer init\r\n");
    #endif
    NameServer ns;
    TID_NS = MyTid();
    int tid, err;
    NameMessage msg;
    #if DEBUG
    bwprintf(COM2, "NameServer (%d) begin loop", TID_NS);
    #endif
    FOREVER { 
        err = Receive(&tid, (void *) &msg, sizeof(msg));
        if (err){
            #if DEBUG
            bwprintf(COM2, "NameServer error: %d\r\n ", err);
            bwprintf(COM2, "Message: %d %s %d\r\n", msg.id, msg.name, msg.tid);
            #endif
            msg.tid = err;
        } else {
            switch (msg.id) {
            case MESSAGE_WHOIS:
                #if DEBUG
                bwprintf(COM2, "NameServer found %s as %d\r\n ", msg.name, ns.names[hash(msg.name)]);
                #endif
                msg.tid = ns.names[hash(msg.name)]; // TODO errors
                break;
            case MESSAGE_REGAS:
                #if DEBUG
                bwprintf(COM2, "NameServer registered %s as %d\r\n ", msg.name, tid);
                #endif
                ns.names[hash(msg.name)] = tid;
                msg.tid = 0;
                break;
            default:
                #if DEBUG
                bwprintf(COM2, "NameServer received invalid argument %d\r\n ", msg.id);
                #endif
                msg.tid = ERR_INVALID_ARGUMENT;
                break;
            }
        }
        Reply(tid, (void*) &msg, sizeof(msg));
    }
}

int RegisterAs(char * name) {
    NameMessage msg;
    msg.id = MESSAGE_REGAS;
    memcpy(msg.name, name, MAXNAMESIZE);
    Send(TID_NS, (void *) &msg, sizeof(msg), (void*) &msg, sizeof(msg));
    return msg.tid;
}

int WhoIs(char * name){
    NameMessage msg;
    msg.id = MESSAGE_WHOIS;
    memcpy(msg.name, name, MAXNAMESIZE);
    Send(TID_NS, (void *) &msg, sizeof(msg), (void*) &msg, sizeof(msg));
    return msg.tid;
}

