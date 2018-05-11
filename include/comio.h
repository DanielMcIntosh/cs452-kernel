#ifndef COMIO_H
#define COMIO_H

int selectchan(int channel, int** flags, int** data);

int comio_put_ready(int channel);
int putc(int channel, char c);
int getc(int channe, char *c);

#endif
