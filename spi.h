#ifndef SPI_H
#define SPI_H

#include <stdbool.h>
#include "utypes.h"

void spi_init();
uchar spi_exchange(uchar tx_data);
void spi_read(uchar* read_buffer, int data_size);
bool spi_transfer_finished();
void spi_flush();

#endif //SPI_H