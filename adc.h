#ifndef ADC_H
#define ADC_H

void adc_init();
void adc_convert_begin();
unsigned char* adc_get_data();
void adc_conversion_on(unsigned int period);
void adc_conversion_off();

#endif //ADC_H
