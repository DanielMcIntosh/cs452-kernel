#ifndef NAME_H
#define NAME_H

#define MAXNAMESIZE 8
#define NUM_NAMES 100

extern int TID_NS;

void __attribute__((noreturn)) task_nameserver();
int RegisterAs(const char * name);
int WhoIs(const char * name);
const char *NameLookup(int tid, char *result_buf);

#endif
