#include "msp430fr2476.h"
#include "utypes.h"


/**************************************************************************
 *
 * SPI аналогичен UART только чтение-отправление идут всегда одновременно и для того чтобы
 * прочитать символ нужно какой-то символ отправить. Если отправлять нечего то для чтения
 * оправляют нулевой байт (0x00)
 *
 * Поскольку засыпание и просыпание процессора происходят "медленно" в сравнении со скоростью передачи SPI
 * то прием и отправку данных по SPI делаем не используя прерывания.
 */

#define NULL 0x00

#define SPI_RX_BUFFER  UCB1RXBUF //spi receive buffer
#define SPI_TX_BUFFER  UCB1TXBUF //spi transmit buffer


//The UCBUSY flag in UCBxSTAT is set while the USCI (SPI) is receiving or sending
#define SPI_BUSY_FLAG (UCB1STATW & UCBUSY)

#define SPI_RX_INTERRUPT_ENABLE()  (UCB1IE |= UCRXIE)
#define SPI_TX_INTERRUPT_ENABLE()  (UCB1IE |= UCTXIE)
#define SPI_RX_INTERRUPT_DISABLE() (UCB1IE &= ~UCRXIE)
#define SPI_TX_INTERRUPT_DISABLE() (UCB1IE &= ~UCTXIE)
#define SET_FR_8MHZ()   (UCB0BR0 |= 0x02)
#define SET_FR_2MHZ()   (UCB0BR0 |= 0x08)

void spi1_set_fr_8mhz() {
    //setting SPI frequency to 16/2=8Mhz
    UCB0BRW |= 0x02;
}

void spi1_set_fr_2mhz() {
    UCB0BRW |= 0x08; //SPI speed = 2Mhz for programming
}

void spi1_init() {
    UCB1CTLW0 |= UCSWRST;                   //Stopping SPI
    UCB1CTLW0 |= (UCMST + UCMSB + UCSYNC);  //SPI 3-wire, master mode, 8 bits per byte, MSB first
    UCB1CTLW0 |= UCSSEL_2;                  //Clock source SMCLK
    //setting SPI frequency to 16384/8=2.048Mhz
    UCB1BRW |= 0x08; //SPI frequency set to 16/8 = 2Mhz
    UCB1STATW = 0x00;                       //Resetting all SPI statistics flags and disable the transmitter feed into receiver
    //configuring ports
    //3.5 - SCLK, 3.2 - MOSI, 3.6 - MISO
    //P1REN &= ~(BIT1 + BIT2 + BIT3);
    P3DIR |= (BIT2 + BIT5);
    P3DIR &= ~(BIT6);
    P3SEL0 |= (BIT2 + BIT5 + BIT6);         //Connecting pins to USCIB1 module
    SPI_RX_INTERRUPT_DISABLE();
    SPI_TX_INTERRUPT_DISABLE();
    UCB1CTLW0 &= ~UCSWRST;                  //Releasing SPI
}

/**
 * Блокирующая операция отправки и получения!
 * Отправляет 1 байт, ждет получения 1 байта и возвращает его.
 */
uchar spi1_transfer(uchar tx_data) {
    __delay_cycles(32);
    while (SPI_BUSY_FLAG); // Wait for TXBUF ready
    SPI_TX_BUFFER = tx_data;         // Send data
    while (SPI_BUSY_FLAG);   // Wait for TX and RX to complete
    uchar rx_data = SPI_RX_BUFFER; // Read received data
    return rx_data;
    __delay_cycles(32);
}


/**
 * Блокирующая операция чтения заданного числа элементов по SPI.
 * Чтобы получить данные будут отправлено соответсвуюющее количество нулей
 * Выйдем их метода только когда все элементы будут прочитаны.
 */
void spi1_read(uchar* read_buffer, int data_size) {
    while(data_size > 0){
        while (SPI_BUSY_FLAG); // Wait for TXBUF ready
        SPI_TX_BUFFER = NULL;  // Send NULL
        while (SPI_BUSY_FLAG); // Wait for TX and RX to complete
        *read_buffer++ = SPI_RX_BUFFER; // Save data from TX_BUFFER to read_buffer
        data_size--;
    }
}



