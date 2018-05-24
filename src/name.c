// big TODO
#include <name.h>
#include <kernel.h>
#include <syscall.h>
#include <message.h>
#include <debug.h>
#include <err.h>
#include <util.h>

typedef struct nameserver {
    int names[NUM_NAMES]; //the worlds dankest hashtable
    // TODO: Write this as a trie instead?
    // TODO
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
    NameServer ns;
    TID_NS = MyTid();
    int tid, err;
    NameMessage msg;
    FOREVER { 
        err = Receive(&tid, (void *) &msg, sizeof(msg));
        if (err){
            msg.tid = err;
            Reply(tid, (void *) &msg, sizeof(msg));
        }
        switch (msg.id) {
        case MESSAGE_WHOIS:
            msg.tid = ns.names[hash(msg.name)]; // TODO errors
            break;
        case MESSAGE_REGAS:
            ns.names[hash(msg.name)] = tid;
            msg.tid = 0;
            break;
        default:
            msg.tid = ERR_INVALID_ARGUMENT;
            break;
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

