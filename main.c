#include "msp430fr2476.h"
#include <stdbool.h>
#include "uart.h"
#include "core_inits.h"
#include "commands.h"
#include "databatch.h"
#include "utils.h"
#include "interrupts.h"
#include "leds.h"
#include "bluetooth.h"
#include "acc.h"

static bool acc_available = false;
static bool adc_available = true;


volatile bool interrupt_flag;

int main(void) {
    stop_watchdog();
    io_init();
    clock_init();
    uart_init();
    LEDS_INIT();
    databatch_init(adc_available, acc_available);
    //bluetooth_init();
    INTERRUPTS_ENABLE();
    while(1){
        while (interrupt_flag) {
            interrupt_flag = false;
//            acc_handle_interrupt();
            commands_process();
            databatch_process();
        }
        // need to read the interrupt flag again without allowing any new interrupts:
        INTERRUPTS_DISABLE();
        if (interrupt_flag == false) {
            SLEEP_WITH_ENABLED_INTERRUPTS(); // an interrupt will cause a wake up and run the while loop
        }
        INTERRUPTS_ENABLE();
    }
}
