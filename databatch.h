#ifndef DATABATCH_H
#define DATABATCH_H

void databatch_init(bool adc_available1, bool acc_available1);
void databatch_start_recording(uchar* ads_dividers);
void databatch_stop_recording();
void databatch_process();

#endif //DATABATCH_H
