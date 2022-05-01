#include "msp430fr2476.h"
#include <stdbool.h>
#include "utypes.h"
#include "interrupts.h"
#include "utils.h"

#define NULL 0x00

#define SPI_RX_BUFFER  UCB0RXBUF //spi receive buffer
#define SPI_TX_BUFFER  UCB0TXBUF //spi transmit buffer

//The UCBUSY flag in UCBxSTAT is set while the USCI (SPI) is receiving or sending
#define SPI_BUSY_FLAG (UCB0STATW & UCBUSY)

#define SPI_RX_INTERRUPT_ENABLE()  (UCB0IE |= UCRXIE)
#define SPI_TX_INTERRUPT_ENABLE()  (UCB0IE |= UCTXIE)
#define SPI_RX_INTERRUPT_DISABLE() (UCB0IE &= ~UCRXIE)
#define SPI_TX_INTERRUPT_DISABLE() (UCB0IE &= ~UCTXIE)

void spi0_init() {
    UCB0CTLW0 |= UCSWRST;                   //Stopping SPI
    UCB0CTLW0 |= (UCMST + UCMSB + UCSYNC +UCCKPL);  //SPI 3-wire, master mode, 8 bits per byte, MSB first, clock high-to low
    UCB0CTLW0 |= UCSSEL_2;                  //Clock source SMCLK
    UCB0BRW |= 0x02; //SPI frequency set to 16/2 = 8Mhz
    UCB0STATW = 0x00;                       //Resetting all SPI statistics flags and disable the transmitter feed into receiver
    //configuring ports
    //1.1 - SCLK, 1.2 - MOSI, 1.3 - MISO
    P1DIR |= (BIT1 + BIT2);
    P1DIR &= ~(BIT3);
    P1SEL0 |= (BIT1 + BIT2 + BIT3);         //Connecting pins to USCIB1 module
    UCB0CTLW0 &= ~UCSWRST;                  //Releasing SPI
    SPI_RX_INTERRUPT_DISABLE();
    SPI_TX_INTERRUPT_DISABLE();
}


/**
 * Блокирующая операция отправки и получения одного байта!
 * Transmit and receive one byte by the SPI
 * Отправляет 1 байт, ждет получения 1 байта и возвращает его.
 */
uchar spi0_transfer(uchar tx_data) {
    while (SPI_BUSY_FLAG); // Wait for TXBUF ready
    SPI_TX_BUFFER = tx_data;         // Send data
    while (SPI_BUSY_FLAG);   // Wait for TX and RX to complete
    uchar rx_data = SPI_RX_BUFFER; // Read received data
    return rx_data;
}

/**
 * Блокирующая операция отправки данных из массива.
 * Ждет пока будут отвравлены все данные
 */
void spi0_transmit(uchar* data, int data_size){
    uchar rx_data;
    while(data_size > 0){
        while (SPI_BUSY_FLAG); // Wait for TXBUF ready
        SPI_TX_BUFFER = *data++;         // Send data
        while (SPI_BUSY_FLAG);   // Wait for TX and RX to complete
        rx_data = SPI_RX_BUFFER; // Read received data
        data_size--;
    }
}

/**
 * Блокирующая операция чтения данных и сохранения их в массив.
 * Ждет пока все будут считаны
 */
void spi0_read(uchar* read_buffer, int data_size) {
    while(data_size > 0){
        while (SPI_BUSY_FLAG); // Wait for TXBUF ready
        SPI_TX_BUFFER = 0x00;         // Send NULL
        while (SPI_BUSY_FLAG);   // Wait for TX and RX to complete
        *read_buffer++ = SPI_RX_BUFFER; // Read received data
        data_size--;
    }
}

