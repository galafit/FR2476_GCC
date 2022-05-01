#ifndef ACC_H
#define ACC_H

#include "utypes.h"

void acc_init();
void acc_send(uchar* data, int data_size);
void acc_test();
void acc_receive(uchar* data, int data_size);
void acc_read();
void acc_stop_reading();
unsigned char* acc_get_data();
void acc_handle_interrupt();

#endif //ACC_H
