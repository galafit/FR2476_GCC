#include "msp430fr2476.h"
#include "interrupts.h"

#define ADS_NUMBER_OF_CHANNELS 1
#define CONVERSIONS_PER_BATCH 128

static unsigned char adc_channels[2] = {6, 7}; //ADC channels for conversion
static int adc_index = 0;
// двойная буфферизация
static unsigned long adc_accumulator_0[ADS_NUMBER_OF_CHANNELS];
static unsigned long adc_accumulator_1[ADS_NUMBER_OF_CHANNELS];
static unsigned long* adc_accumulators[2] = {adc_accumulator_0, adc_accumulator_1};
static unsigned char i = 0;            //ADC accumulator iterator
static unsigned char adc_accumulator_index = 0;
static unsigned int adc_data_prepared[ADS_NUMBER_OF_CHANNELS];
static unsigned long* adc_accumulator = adc_accumulator_0;
static unsigned int adc_sum_cnt = 0;

void adc_init(){
  //Setup TimerB as ADC trigger source
  TB0EX0=TB0CTL = 0;                             //Switching the timer off, interrupt on
  TB0CTL |= (TBSSEL_2 + ID_3);                   //Clocking timer from SMCLK/8, timer stopped
  TB0CCR0 = 0x00;                                //Timer period
  TB0CCTL0 |= CCIE;                              //Compare0 module interrupt enabled

  // P1.6, P1.7 - ADC pins
  P1REN &= ~(BIT6 + BIT7);
  P1SELC |= (BIT6 + BIT7);
  SYSCFG2 |= (BIT6 + BIT7);        //Activate ADC module on the pins

  // ADC Input positive reference VEREF+  P1.0
  // ADC Input negative reference VEREF-  P1.2
    P1REN &= ~(BIT0);
    P1SELC |= (BIT0);

  //Configuring ADC
//  PMMCTL2 |= (INTREFEN);                       //Internal reference 1.5V
  ADCCTL0=ADCCTL1=ADCCTL2 = 0;                   //Ensuring that the ADC is off
  ADCCTL0 |= ADCMSC;                             //sample and hold = 4clk, multiple conversion
  ADCCTL1 |= (ADCSHP + ADCSSEL_2 + ADCDIV_2);    //TIMER Conversion is triggered manually, ADC clock source - SMCLK/3, single channel single conversion
  ADCCTL2 |= ADCRES_2;                           //12 bit resolution
//  ADCMCTL0 |= (ADCSREF_1 + adc_channels[adc_index]);           //Employing the internal reference, start conversion from the first channel in the list
  ADCMCTL0 |= (ADCSREF_3 + adc_channels[adc_index]);
  ADCIE |= ADCIE0;                               //Activate interrupt
  ADCCTL0 |= (ADCON);
}

/* --------------------- Конвертация по 4м каналам -------------------- */

/*
 * This function turns on continuous timer triggered conversion.
 * Period is defined by the "period" variable.
 */
void adc_conversion_on(unsigned int period){
    TB0CCR0 = period;                            //Loading timer ticking period
    TB0CTL |= (MC_1 + TBIE);                     //Timer is in Up mode, interrupt on
}

void adc_conversion_off(){
    TB0CCR0 = 0x00;                              //Zeroing timer period
    TB0CTL &= ~(MC_1 + TBIE);                    //Timer is stopped, interrupt off
}

void adc_convert_begin(){
    ADCCTL0 |= ADCENC;
    ADCCTL0 |= ADCSC;           //Start conversion
}

/* -------------------------------------------------------------------------- */
// TODO убедиться что оцифрованные данные по прерыванию не прилетят во время преключения
// двойного буффера. Если такое возможно поставить защиту
unsigned char* adc_get_data(){
    //После того, как все данные оцифровались, обнуляем счетчик суммации
    //И преобразуем longs в ints, для отсылки на PC
    for(int i = 0; i < ADS_NUMBER_OF_CHANNELS; i++){
//        if(adc_accumulator[i] > 65535) adc_accumulator[i] = 65535;  //Clipping the sample value
        adc_data_prepared[i] = (unsigned int)(adc_accumulator[i] >> 4);
    }
    unsigned char* data = (unsigned char*) adc_data_prepared;
    // переключаемся на второй буффер и обнуляем его значения
    adc_accumulator = adc_accumulators[(i++ & 1)];
    for (int i = 0; i < ADS_NUMBER_OF_CHANNELS; ++i) {
        adc_accumulator[i] = 0;
    }

    adc_sum_cnt = 0; //Marking that the adc batch is prepared and we are ready for the next conversion
    return data;
}


__attribute__((interrupt(ADC_VECTOR)))
void ADC_ISR(void){
    unsigned int value;
    switch(__even_in_range (ADCIV, 0x0C)){
        case 0x0C:
            // нужно обязательно считать данные из буффера ADCMEM0 чтобы выйти из прерывания
            //Reading the current conversion
            value = ADCMEM0;
            if(adc_sum_cnt < CONVERSIONS_PER_BATCH){
                ADCCTL0 &= ~(ADCENC);                      //Turning the ADC off before changing the channel
                adc_accumulator[adc_index++] += (unsigned long)value;    //Reading the current conversion
                //Making sure we are in the range
                if(adc_index > (ADS_NUMBER_OF_CHANNELS-1)){
                    adc_sum_cnt++;
                    adc_index = 0;
                }
                ADCMCTL0 = (ADCSREF_3 + adc_channels[adc_index]);   //Choosing the next channel
                //If we have converted all the planned channels, stop here
                //and wait for the next turn (timer will trigger a new conversion series)
                if(adc_index > 0){
                    adc_convert_begin();
                }
            }
            break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}

__attribute__((interrupt(TIMER0_B0_VECTOR)))
void TIMERB0_ISR(void){
    switch(__even_in_range (TB0IV, 0x0E)){
    case 0x0E:
        adc_convert_begin();                //Initiate the first conversion
        break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}
