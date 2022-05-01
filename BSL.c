#include "msp430fr2476.h"
#include "interrupts.h"
#include "adc.h"
#include "acc.h"
#include "ads1292.h"
#include "utypes.h"

void call_BSL(){
    //Останавливаем всю переферию и запускаем загрузчик
    //!!!__!!! Обязательно надо здесь включить блютус на частоте 9600, even parity!
    TA2CTL |= TACLR;
    UCA1CTL1 |= UCSWRST;
    ads_stop_recording();
    adc_conversion_off();
    acc_stop_reading();
    INTERRUPTS_DISABLE();
    ((void (*)())0x1000)();
}
