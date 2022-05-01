#ifndef LEDS_H
#define LEDS_H

#include "msp430fr2476.h"

#define LEDS_INIT() P5REN &= ~(BIT0+BIT1); P4REN &= ~BIT7; P5DIR |= (BIT0+BIT1); P4DIR |= BIT7; P5OUT &= ~(BIT0+BIT1); P4OUT &= ~BIT7

#define LED1_ON()  P5OUT |= BIT0
#define LED2_ON()  P5OUT |= BIT1
#define LED3_ON()  P4OUT |= BIT7
#define LED1_OFF()  P5OUT &= ~BIT0
#define LED2_OFF()  P5OUT &= ~BIT1
#define LED3_OFF()  P4OUT &= ~BIT7
#define LED1_SWITCH()  P5OUT ^= BIT0
#define LED2_SWITCH()  P5OUT ^= BIT1
#define LED3_SWITCH()  P4OUT ^= BIT7


#endif //LEDS_H
