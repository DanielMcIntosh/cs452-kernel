#ifndef NAME_H
#define NAME_H

#define MAXNAMESIZE 8
#define NUM_NAMES 100

extern int TID_NS;

void task_nameserver();
int RegisterAs(char * name);
int WhoIs(char * name);

#endif
