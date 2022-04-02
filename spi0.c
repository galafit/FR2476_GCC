#include "msp430fr2476.h"
#include <stdbool.h>
#include "utypes.h"
#include "interrupts.h"
#include "utils.h"

#define ACC_CS BIT1
#define ACC_SELECT() (P3OUT &= ~ACC_CS)
#define ACC_DESELECT() (P3OUT |= ACC_CS)

#define NULL 0x00

#define SPI0_RX_BUFFER  UCB0RXBUF //spi receive buffer
#define SPI0_TX_BUFFER  UCB0TXBUF //spi transmit buffer

//The UCBUSY flag in UCBxSTAT is set while the USCI (SPI) is receiving or sending
#define SPI0_BUSY_FLAG (UCB0STATW & UCBUSY)

#define SPI0_RX_INTERRUPT_ENABLE()  (UCB0IE |= UCRXIE)
#define SPI0_TX_INTERRUPT_ENABLE()  (UCB0IE |= UCTXIE)
#define SPI0_RX_INTERRUPT_DISABLE() (UCB0IE &= ~UCRXIE)
#define SPI0_TX_INTERRUPT_DISABLE() (UCB0IE &= ~UCTXIE)

/*---- ссылка на буфер куда будут сохраняться поступающие данные-----*/
static volatile uchar* spi0_rx_data;
static volatile int spi0_rx_data_size;

/*---- ссылка на буфер из которого будут отправляться данные-----*/
static volatile uchar* spi0_tx_data;
static volatile int spi0_tx_data_size;

static volatile bool read_available;  //true поступающие данные сохраняются в буфер spi_rx_data
static volatile bool transmit_available; //true данные отправляются из буффера spi_tx_data, false вместо данных отправляется NULL

void spi0_init() {
    UCB0CTLW0 |= UCSWRST;                   //Stopping SPI
    UCB0CTLW0 |= (UCMST + UCMSB + UCSYNC +UCCKPL);  //SPI 3-wire, master mode, 8 bits per byte, MSB first, clock high-to low
    UCB0CTLW0 |= UCSSEL_2;                  //Clock source SMCLK
    //setting SPI frequency to 16384/8=2.048Mhz
    UCB0BRW |= 0x02;                        //SPI frequency set to 16/2 = 8Mhz
    UCB0STATW = 0x00;                       //Resetting all SPI statistics flags and disable the transmitter feed into receiver
    //configuring ports
    //1.1 - SCLK, 1.2 - MOSI, 1.3 - MISO
    P1DIR |= (BIT1 + BIT2);
    P1DIR &= ~(BIT3);
    P1SEL0 |= (BIT1 + BIT2 + BIT3);         //Connecting pins to USCIB1 module
    UCB0CTLW0 &= ~UCSWRST;                  //Releasing SPI
}

/**
 * Блокирующая операция отправки и получения одного байта! Transmit and receive one byte by the SPI
 * Отправляет 1 байт, ждет получения 1 байта и возвращает его.
 * Не применять одновременно с неблокирующи  SPI обменом
 * использующим прерывания! Результат не предсказуем
 * PS этот метод в ардуино библиотеках обычно называют spi_transfer
 */
uchar spi0_exchange(uchar tx_data) {
    SPI0_RX_INTERRUPT_DISABLE();
    SPI0_TX_INTERRUPT_DISABLE();
    ACC_SELECT();
    //while (!SPI_TX_FLAG); // Wait for TXBUF ready
    while (SPI0_BUSY_FLAG); // Wait for TXBUF ready
    SPI0_TX_BUFFER = tx_data;         // Send data
    while (SPI0_BUSY_FLAG);   // Wait for TX and RX to complete
    uchar rx_data = SPI0_RX_BUFFER; // Read received data
    return rx_data;
}

/**
 * Блокирующая операция отправки данных из массива.
 * Ждет пока будут отвравлены все данные
 */
void spi0_transmit(uchar* data, int data_size){
    SPI0_RX_INTERRUPT_DISABLE();
    SPI0_TX_INTERRUPT_DISABLE();
    ACC_SELECT();
    uchar rx_data;
    while(data_size > 0){
        while (SPI0_BUSY_FLAG); // Wait for TXBUF ready
        SPI0_TX_BUFFER = *data++;         // Send data
        while (SPI0_BUSY_FLAG);   // Wait for TX and RX to complete
        rx_data = SPI0_RX_BUFFER; // Read received data
        data_size--;
    }
    ACC_DESELECT();
}

/**
 * Блокирующая операция чтения данных и сохранения их в массив.
 * Ждет пока все будут считаны
 */
void spi0_read1(uchar* read_buffer, int data_size) {
    SPI0_RX_INTERRUPT_DISABLE();
    SPI0_TX_INTERRUPT_DISABLE();
    ACC_SELECT();
    while(data_size > 0){
        while (SPI0_BUSY_FLAG); // Wait for TXBUF ready
        SPI0_TX_BUFFER = 0x00;         // Send NULL
        while (SPI0_BUSY_FLAG);   // Wait for TX and RX to complete
        *read_buffer++ = SPI0_RX_BUFFER; // Read received data
        data_size--;
    }
    ACC_DESELECT();
}

/**
 * Неблокирующее получение заданного числа элементов по SPI.
 * Чтобы их получить будут отправлено соответсвуюющее количество нулей
 * Для чтения больших массивов данных
 */
void spi0_read(uchar* read_buffer, int data_size) {
    SPI0_RX_INTERRUPT_DISABLE();
    SPI0_TX_INTERRUPT_DISABLE();
    ACC_SELECT();
    spi0_rx_data = read_buffer;
    spi0_rx_data_size = data_size;
    spi0_tx_data_size = data_size;
    transmit_available = false;
    read_available = true;
    UCB0IFG |= UCTXIFG;         //Trigger first Tx interrupt
    SPI0_RX_INTERRUPT_ENABLE();  // Enable Receive  interrupt
    SPI0_TX_INTERRUPT_ENABLE();  // Enable Transmit  interrupt
}


//***Combined SPI0 Tx/Rx interrupt vector***
__attribute__((interrupt(USCI_B0_VECTOR)))
void USCI_B0_ISR(void){
    switch(__even_in_range (UCB0IV, 8)){  //this intrinsic tells the compiler to omit odd checks, so the code is faster
    uchar ch;
    case 0x02:
         //Rx
        // Прочитать символ из буфера-приемника
         ch = SPI0_RX_BUFFER;
         if(spi0_rx_data_size > 0) {
             if(read_available) {
                 *spi0_rx_data++ = ch; // положить символ в буффер для получения данных
             }
             spi0_rx_data_size--;
         }
         if (spi0_rx_data_size <= 0) {  // ожидаемое количество данных уже принято
             // Выключаем прерывание на прием по SPI
             ACC_DESELECT();
             SPI0_RX_INTERRUPT_DISABLE();
         }
       break;
    case 0x04:
          //Tx
          if (spi0_tx_data_size <= 0) { // нечего передавать
              // Выключаем прерывание на передачу SPI
              SPI0_TX_INTERRUPT_DISABLE();
          } else {
              if(transmit_available) {
                  SPI0_TX_BUFFER = *spi0_tx_data++; // отправляем данные
              } else {
                  SPI0_TX_BUFFER = NULL; // отправляем ноль чтобы прочитать данные
              }
              spi0_tx_data_size--;
          }
        break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}


