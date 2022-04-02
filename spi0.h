#ifndef SPI0_H
#define SPI0_H

#include <stdbool.h>
#include "utypes.h"

void spi0_init();
uchar spi0_exchange(uchar tx_data);
void spi0_read(uchar* read_buffer, int data_size);
void spi0_read1(uchar* read_buffer, int data_size);
void spi0_transmit(uchar* data, int data_size);

#endif //SPI0_H