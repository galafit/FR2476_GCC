#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include "utypes.h"

void uart_init();
bool uart_read(uchar* chp);
void uart_transmit(uchar *data, int data_size);
void uart_flush();
void uart_rx_fifo_erase();

#endif //UART_H



