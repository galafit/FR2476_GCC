#ifndef SPI1_H
#define SPI1_H

#include "utypes.h"

void spi1_init();
uchar spi1_transfer(uchar tx_data);
void spi1_read(uchar* read_buffer, int data_size);

#endif //SPI1_H
