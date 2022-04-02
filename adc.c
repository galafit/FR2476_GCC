#include "msp430fr2476.h"
#include "interrupts.h"

#define ADC_NUMBER_OF_CHANNELS 4
#define CONVERSIONS_PER_BATCH 16

static unsigned char adc_channels[4] = {4, 5, 6, 7}; //АЦП каналы для конвертации
static int adc_index = 0;
// двойная буфферизация
static unsigned long adc_accumulator_0[ADC_NUMBER_OF_CHANNELS];
static unsigned long adc_accumulator_1[ADC_NUMBER_OF_CHANNELS];
static unsigned long* adc_accumulators[2] = {adc_accumulator_0, adc_accumulator_1};
static unsigned char i = 0;            //Итератор АЦП аккумулятора
static unsigned char adc_accumulator_index = 0;
static unsigned int adc_data_prepared[ADC_NUMBER_OF_CHANNELS];
static unsigned long* adc_accumulator = adc_accumulator_0;
static unsigned int adc_sum_cnt = 0;

void adc_init(){
  //АЦП будет запускаться от TimerB
  TB0EX0=TB0CTL = 0;                             //Выключаем таймер
  TB0CTL |= (TBSSEL_2 + ID_3);                   //Тактируем от SMCLK/8
  TB0CCR0 = 0x00;                                //Обнуляем период таймера
  TB0CCTL0 |= CCIE;                              //Включаем один из модулей сравнения (для срабатывания прерывания таймера)

  //P1.4, P1.5, P1.6, P1.7 - пины АЦП
  P1REN &= ~(BIT4 + BIT5 + BIT6 + BIT7);
  P1SELC |= (BIT4 + BIT5 + BIT6 + BIT7);
  SYSCFG2 |= (BIT4 + BIT5 + BIT6 + BIT7);        //Активируем АЦП на этих пинах

  //Конфигурируем АЦП
  PMMCTL2 |= (INTREFEN);                         //Internal reference 1.5V
  ADCCTL0=ADCCTL1=ADCCTL2 = 0;                   //Отключаем АЦП
  ADCCTL0 |= ADCMSC;                             //Sample and hold = 4clk, multiple conversion
  ADCCTL1 |= (ADCSHP + ADCSSEL_2 + ADCDIV_2);    //TIMER Conversion is triggered manually, ADC clock source - SMCLK/3, single channel single conversion
  ADCCTL2 |= ADCRES_2;                           //12 bit resolution
  ADCMCTL0 |= (ADCSREF_1 + adc_channels[adc_index]);           //Employing the internal reference, start conversion from the first channel in the list
  ADCIE |= ADCIE0;                               //Activate interrupt
  ADCCTL0 |= (ADCON);
}

/* --------------------- Конвертация по 4м каналам -------------------- */

void adc_conversion_on(unsigned int period){
    TB0CCR0 = period;                 //Загружаем в таймер период срабатывания
    TB0CTL |= (MC_1 + TBIE);          //Переводим таймер в режим Up, включаем прерывания
}

void adc_conversion_off(){
    TB0CCR0 = 0x00;                   //Обнуляем период таймера
    TB0CTL &= ~(MC_1 + TBIE);         //Выключаем таймер и прерывания таймера
}

void adc_convert_begin(){
    ADCCTL0 |= ADCENC;
    ADCCTL0 |= ADCSC;                 //Запускаем конверсию
}

/* -------------------------------------------------------------------------- */
// TODO убедиться что оцифрованные данные по прерыванию не прилетят во время преключения
// двойного буффера. Если такое возможно поставить защиту
unsigned char* adc_get_data(){
    //После того, как все данные оцифровались, обнуляем счетчик суммации
    //И преобразуем longs в ints, для отсылки на PC
    for(int i = 0; i < ADC_NUMBER_OF_CHANNELS; i++){
        if(adc_accumulator[i] > 65535) adc_accumulator[i] = 65535;  //Clipping the sample value
        adc_data_prepared[i] = (unsigned int)adc_accumulator[i];
    }
    unsigned char* data = (unsigned char*) adc_data_prepared;
    adc_accumulator = adc_accumulators[(i++ & 1)];
    // переключаемся на второй буффер и обнуляем его значения
    /*if(adc_accumulator_index == 0) {
        adc_accumulator_index = 1;
        adc_accumulator = adc_accumulator_1;

    } else {
        adc_accumulator_index = 0;
        adc_accumulator = adc_accumulator_0;
    }
    */
    for (int i = 0; i < ADC_NUMBER_OF_CHANNELS; ++i) {
        adc_accumulator[i] = 0;
    }

    adc_sum_cnt = 0;            //Обозначаем, что пакет сформирован и можно оцифровывать следующие сэмплы
    return data;
}

__attribute__((interrupt(ADC_VECTOR)))
void ADC_ISR(void){
    switch(__even_in_range (ADCIV, 0x0C)){
    case 0x0C:
        if(adc_sum_cnt >= CONVERSIONS_PER_BATCH){
            //If all the data has been converted, but not collected yet
            //write it to a dummy int in order to avoid brainfuck with the
            //ADC interrupt triggering endlessly
            int dummy = ADCMEM0;
         goto exit;
        }
        ADCCTL0 &= ~(ADCENC);                      //Отключаем АЦП перед тем, как поменять канал
        adc_accumulator[adc_index++] += (unsigned long)ADCMEM0;    //Читаем/суммируем оцифрованный сэмпл
        //Если все каналы сконвертированы, сбрасываем счетчик каналов
        if(adc_index > (ADC_NUMBER_OF_CHANNELS-1)){
            adc_sum_cnt++;
            adc_index = 0;
        }
        ADCMCTL0 = (ADCSREF_1 + adc_channels[adc_index]);   //Переключаемся на следующий канал
        //Если мы сконвертировали данные со всех нужных каналов, останавливаемся здесь
        //и ждем следующей команды на запуск от таймера
        if(adc_index > 0){
            adc_convert_begin();
        }
        break;
    }
exit:
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}

__attribute__((interrupt(TIMER0_B0_VECTOR)))
void TIMERB0_ISR(void){
    switch(__even_in_range (TB0IV, 0x0E)){
    case 0x0E:
        adc_convert_begin();                //Запускаем самое первое преобразование
        break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}
