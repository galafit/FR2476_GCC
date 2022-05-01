#ifndef SPI0_H
#define SPI0_H

#include <stdbool.h>
#include "utypes.h"

void spi0_init();
uchar spi0_transfer(uchar tx_data);
void spi0_read(uchar* read_buffer, int data_size);

#endif //SPI0_H
