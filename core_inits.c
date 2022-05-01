#include <msp430fr2476.h>
#include "utils.h"

//This function stops the watchdog timer
void stop_watchdog(){
    WDTCTL = WDTPW | WDTHOLD;
}

//Ensuring that at startup all the pins are in a certain condition (inputs, tied to GND)
//!!___!!! Tied to GND as of yet
void io_init(){
    PM5CTL0 &= ~LOCKLPM5;           // Disable the GPIO power-on default high-impedance mode
    P1SEL0=P2SEL0=P3SEL0=P4SEL0=P5SEL0=P6SEL0 = 0x00;
    P1SEL1=P2SEL1=P3SEL1=P4SEL1=P5SEL1=P6SEL1 = 0x00;
    P1REN=P2REN=P3REN=P4REN=P5REN=P6REN = 0xff;
    P1DIR=P2DIR=P3DIR=P4DIR=P5DIR=P6DIR = 0x00;
    P1OUT=P2OUT=P3OUT=P4OUT=P5OUT=P6OUT = 0x00;
    //Deselecting ACC
    P4OUT |= BIT3;
    P4REN &= ~BIT3;
    P4DIR |= BIT3;
    //Deselecting ADS
    P4OUT |= BIT1;
    P4REN &= ~BIT1;
    P4DIR |= BIT1;
}

void clock_init(){
  //Setting pins 2.0 and 2.1 for Xtal operation
  P2REN &= ~(BIT0 + BIT1);          //No need for any pullup on Xtal pins
  P2DIR &= ~(BIT1);                 //Xin
  P2DIR |= BIT0;                    //Xout
  P2SEL0 |= (BIT0 + BIT1);          //These pins are set for the Xtal
  //!!!__!!!  Perhaps should lower the Xtal Drive strength for lower power consumption (CSCTL6 register XT1DRIVE bits values 0-3)
  CSCTL6 &= ~(XT1AUTOOFF + XT1BYPASS);          //!!!__!!! switching off crystal auto-shutdown feature
  //Giving xtal time to stabilize
  while(CSCTL7 & XT1OFFG){
    CSCTL7 &= ~(XT1OFFG);           //Clearing Xtal fault flag
    SFRIFG1 &= ~(OFIFG);            //Clearing Oscillator fault interrupt flag
  }
  __bis_SR_register(SCG0);         //Disable FLL
  CSCTL0 = 0;                      //Clearing all the DCO settings to prepare them for automatic FLL control
  CSCTL1 |= DCORSEL_5;             //DCO set for 16MHZ !!___!!! apparently won't be needed
  CSCTL2 &= ~(FLLD);
  CSCTL2 &= ~(0x3ff);              //Clearing field for speed setting
  CSCTL2 |= 499;                   //A magic number to tune DCO FLL to 16,384MHZ
  wait(3);                         //Allow time for FLL settings to apply
  __bic_SR_register(SCG0);
  while(CSCTL7 & FLLUNLOCK){};     //Wait for FLL to stabilize
  CSCTL3 |= REFOLP;                //REFO clock is set to low-power mode
  //CSCTL6 &= ~(XT1DRIVE);          //!!!__!!! Perhaps will use this for reducing Xtal drive and thus, power consumption

  //Setting up TimerA and use its output at pin 3.3 as the clock source for ADS
  P3REN &= ~BIT3;
  P3DIR |= BIT3;
  P3SEL0 |= BIT3;

  TA2CTL |= TACLR;                    //Clearing the timer and stopping it
  TA2CCR0 = 0x04;                     //Loading the timer for 2.049Mhz period
  TA2CCTL1 |= OUTMOD_7;
  TA2CTL |= (TASSEL_2 + MC_3);        //Timer ticks at 1/8 from SMCLK, up\down mode
}




