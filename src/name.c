// big TODO
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

void task_nameserver(){
    #if DEBUG
    bwputstr(COM2, "NameServer init\r\n");
    #endif
    NameServer ns;
    ht_init(&ns.ht);
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
                msg.tid = ht_lookup(&ns.ht, msg.name);
                #if DEBUG
                bwprintf(COM2, "NameServer found %s as %d\r\n ", msg.name, msg.tid);
                #endif
                break;
            case MESSAGE_REGAS:
                #if DEBUG
                bwprintf(COM2, "NameServer registering %s as %d\r\n ", msg.name, tid);
                #endif
                msg.tid = ht_insert(&ns.ht, msg.name, tid);
                #if DEBUG
                bwprintf(COM2, "NameServer registered %s as %d\r\n ", msg.name, tid);
                #endif
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
    memcpy(msg.name, name, MAXNAMESIZE); // TODO legal name checking
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

