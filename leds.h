#ifndef LEDS_H
#define LEDS_H

#include "msp430fr2476.h"

#define LED_INIT() P1DIR |= BIT0; P1OUT &= ~BIT0;
#define LED_ON()  (P1OUT |= BIT0)
#define LED_OFF()  (P1OUT &= ~BIT0)

#define LED1_ON()  (P1OUT |= BIT5)
#define LED2_ON()  (P1OUT |= BIT6)
#define LED3_ON()  (P1OUT |= BIT7)
#define LED1_OFF()  (P1OUT &= ~BIT5)
#define LED2_OFF()  (P1OUT &= ~BIT5)
#define LED3_OFF()  (P1OUT &= ~BIT7)
#define LED1_SWITCH()  (P1OUT ^= BIT5)
#define LED2_SWITCH()  (P1OUT ^= BIT6)
#define LED3_SWITCH()  (P1OUT ^= BIT7)


#endif //LEDS_H
