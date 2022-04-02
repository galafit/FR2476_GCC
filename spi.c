#include "msp430fr2476.h"
#include <stdbool.h>
#include "utypes.h"
#include "leds.h"
#include "interrupts.h"
#include "utils.h"

/**************************************************************************
 *
 * SPI аналогичен UART только чтение-отправление идут всегда одновременно и для того чтобы
 * прочитать символ нужно какой-то символ отправить. Если отправлять нечего то для чтения
 * оправляют нулевой байт (0x00)
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

/*---- ссылка на буфер куда будут сохраняться поступающие данные-----*/
static volatile uchar* spi_rx_data;
static volatile int spi_rx_data_size;

/*---- ссылка на буфер из которого будут отправляться данные-----*/
static volatile uchar* spi_tx_data;
static volatile int spi_tx_data_size;

static volatile bool read_available;  //true поступающие данные сохраняются в буфер spi_rx_data
static volatile bool transmit_available; //true данные отправляются из буффера spi_tx_data, false вместо данных отправляется NULL

void spi_init() {
    UCB1CTLW0 |= UCSWRST;                   //Stopping SPI
    UCB1CTLW0 |= (UCMST + UCMSB + UCSYNC);  //SPI 3-wire, master mode, 8 bits per byte, MSB first
    UCB1CTLW0 |= UCSSEL_2;                  //Clock source SMCLK
    //setting SPI frequency to 16384/8=2.048Mhz
    UCB1BRW |= 0x08;                        //SPI frequency set to 16/8 = 2Mhz
    UCB1STATW = 0x00;                       //Resetting all SPI statistics flags and disable the transmitter feed into receiver
    //configuring ports
    //3.5 - SCLK, 3.2 - MOSI, 3.6 - MISO
    //P1REN &= ~(BIT1 + BIT2 + BIT3);
    P3DIR |= (BIT2 + BIT5);
    P3DIR &= ~(BIT6);
    P3SEL0 |= (BIT2 + BIT5 + BIT6);         //Connecting pins to USCIB1 module
    UCB1CTLW0 &= ~UCSWRST;                  //Releasing SPI
}

/**
 * Блокирующая операция отправки и получения! Transmit and receive one byte by the SPI
 * Отправляет 1 байт, ждет получения 1 байта и возвращает его.
 * Не применять одновременно с неблокирующи  SPI обменом
 * использующим прерывания! Результат не предсказуем
 * PS этот метод в ардуино библиотеках обычно называют spi_transfer
 */
uchar spi_exchange(uchar tx_data) {
    SPI_RX_INTERRUPT_DISABLE();
    SPI_TX_INTERRUPT_DISABLE();
    //while (!SPI_TX_FLAG); // Wait for TXBUF ready
    while (SPI_BUSY_FLAG); // Wait for TXBUF ready
    SPI_TX_BUFFER = tx_data;         // Send data
    while (SPI_BUSY_FLAG);   // Wait for TX and RX to complete
    uchar rx_data = SPI_RX_BUFFER; // Read received data
    return rx_data;
}


/**
 * Неблокирующее получение заданного числа элементов по SPI.
 * Чтобы их получить будут отправлено соответсвуюющее количество нулей
 * Для чтения больших массивов данных
 */
void spi_read(uchar* read_buffer, int data_size) {
    SPI_RX_INTERRUPT_DISABLE();
    SPI_TX_INTERRUPT_DISABLE();
    spi_rx_data = read_buffer;
    spi_rx_data_size = data_size;
    spi_tx_data_size = data_size;
    transmit_available = false;
    read_available = true;
    UCB1IFG |= UCTXIFG;         //Trigger first Tx interrupt
    SPI_RX_INTERRUPT_ENABLE();  // Enable Receive  interrupt
    SPI_TX_INTERRUPT_ENABLE();  // Enable Transmit  interrupt
}

/**
 *  Ждет пока  ассинхронная передача и чтение по SPY будут завершены
 */
void spi_flush() {
    //while(spi_rx_data_size > 0);
}

/**
 * @return true если ассинхронная передача и чтение по SPY завершены
 */
bool spi_transfer_finished() {
  if(spi_rx_data_size <= 0) {
    return true;
  }
  return false;
}


//***Combined SPI Tx/Rx interrupt vector***
int count = 0;
__attribute__((interrupt(USCI_B1_VECTOR)))
void USCI_B1_ISR(void){
    switch(__even_in_range (UCB1IV, 8)){  //this intrinsic tells the compiler to omit odd checks, so the code is faster
    uchar ch;
    case 0x02:
         //Rx
        // Прочитать символ из буфера-приемника
         ch = SPI_RX_BUFFER;
         if (spi_rx_data_size <= 0) {  // ожидаемое количество данных уже принято
             // Выключаем прерывание на прием по SPI
             SPI_RX_INTERRUPT_DISABLE();
         } else {
             if(read_available) {
                 *spi_rx_data++ = ch; // положить символ в буффер для получения данных
             }
             spi_rx_data_size--;
         }
       break;
    case 0x04:
          //Tx
          if (spi_tx_data_size <= 0) { // нечего передавать
              // Выключаем прерывание на передачу SPI
              SPI_TX_INTERRUPT_DISABLE();
          } else {
              if(transmit_available) {
                  SPI_TX_BUFFER = *spi_tx_data++; // отправляем данные
              } else {
                  SPI_TX_BUFFER = NULL; // отправляем ноль чтобы прочитать данные
              }
              spi_tx_data_size--;
          }
        break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}


