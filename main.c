#include "uart.h"
#include "spi.h"
#include "spi0.h"
#include "msp430fr2476.h"
#include "core_inits.h"
#include "commands.h"
#include "ads1292.h"
#include "adc.h"
#include "databatch.h"
#include "utils.h"
#include "interrupts.h"
#include "leds.h"
#include "acc.h"
#include "bluetooth.h"

volatile bool interrupt_flag;

int main(void) {
    stop_watchdog();
    io_init();
    clock_init();
    uart_init();
    spi_init();
    spi0_init();
    ads_init();
    adc_init();
    LED_INIT();
    // передаем в ADS ссылку на функцию из ADC10 которую ads будет вызывать в прерывании DRDY при поступлении данных
    //ads_DRDY_interrupt_callback(adc_convert_begin);
    INTERRUPTS_ENABLE();
    acc_init();
    adc_conversion_on(255); //Эту функцию надо будет поместить там, где стартует АДС
    acc_read();             //Эту функцию надо будет поместить там, где стартует АДС
    //bluetooth_init();
    //ads_start_recording();
    while(1){
        while (interrupt_flag) {
            //LED_ON();
            interrupt_flag = false;
            acc_handle_interrupt();
            commands_process();
            databatch_process();
            //LED_OFF();
        }
        // need to read the interrupt flag again without allowing any new interrupts:
        INTERRUPTS_DISABLE();
        if (interrupt_flag == false) {
            SLEEP_WITH_ENABLED_INTERRUPTS(); // an interrupt will cause a wake up and run the while loop
        }
        INTERRUPTS_ENABLE();

    }
}
