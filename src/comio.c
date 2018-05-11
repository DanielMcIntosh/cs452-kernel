#include <comio.h>
#include <ts7200.h>

int selectchan(int channel, int** flags, int** data){
	switch( channel ) {
	case COM1:
		*flags = (int *)( UART1_BASE + UART_FLAG_OFFSET );
		*data = (int *)( UART1_BASE + UART_DATA_OFFSET );
		break;
	case COM2:
		*flags = (int *)( UART2_BASE + UART_FLAG_OFFSET );
		*data = (int *)( UART2_BASE + UART_DATA_OFFSET );
		break;
	default:
		return 1;
		break;
	}

    return 0;
}

int comio_put_ready(int channel) {
    int * flags, *data;
    int err = selectchan(channel, &flags, &data);
    if (err)
        return err;
    int ready = !(*flags & TXFF_MASK); //&& ((*flags & CTS_MASK) == 0);

    if (channel == COM1)
        ready = ready && (*flags & CTS_MASK);

    return ready;
}

int putc(int channel, char c){
    int * flags, *data;
    int err = selectchan(channel, &flags, &data);
    if (err)
        return err;
    if (!(*flags & TXFF_MASK))
        *data = c;
    else
        return 1;
    
    return 0;
}

int getc(int channel, char *c) {
    int * flags, *data;
    int err = selectchan(channel, &flags, &data);
    if (err)
        return err;
    if (!(*flags & RXFE_MASK))
        *c = *data & 0xff;
    else
        return 1;
    
    return 0;
}
