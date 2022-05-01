#include "msp430fr2476.h"
#include <stdbool.h>
#include "utypes.h"
#include "leds.h"
#include "interrupts.h"
#include "utils.h"

/**
 * Обмен информацией через UART происходит в дуплексном режиме,
 * т.е. передача данных может происходить одновременно с приемом.
 * Для этого в интерфейсе UART есть два сигнала:
 *   TX – выход  для отправки данных (transmitter)
 *   RX – вход для приема данных (receiver)
 * **********************************************************************
 * UART имеет 2 буффера - один для получения и один для отправки данных:
 *
 * UCAxRXBUF Receive buffer register
 * UCAxTXBUF Transmit buffer register
 * **********************************************************************
 * Когда данные поступают в RXBUF или отправляются из TXBUF выставляется соотвествующий флаг в
 * регистре флагов прерываний [INTERRUPT_FLAG_REGISTER_2] (IFG2)
 *
 * Флаг(бит) UCA0RXIFG выставляется когда символ поступил во входной буффер RXBUF
 * Флаг сбрасывается когда символ из RXBUF прочитан
 *
 * Флаг(бит) UCA0TXIFG выставляется когда символ из буфера передачи TXBUF
 * перебрасываются в сдвиговый регистр и TXBUF становится свободен для записи след символа
 * Флаг сбрасывается когда в TXBUF помещается символ
 * Tак же этот флаг выставляется after a PUC or when UCSWRST = 1
 *
 * PS Флаги UCA0RXIFG и UCA0TXIFG устанавливет и сбрасывает система.
 * Ручками это делать не нужно и не правильно!
 * **********************************************************************
 * При установке флагов прерываний срабатывает соответсвующий вектор прерываний.
 * UART имеет 2 вектора прерываний:
 *
 * RX-interrupt срабатывает когда выставлен флаг UCA0RXIFG
 * (данные поступилит во входной буффер RXBUF)
 *
 * TX-interrupt срабатывает когда выставлен флаг UCA0TXIFG
 * (буфер на отправку TXBUF свободен для записи)
 *
 * PS UART прерывания работают по уровню а не по фронту! То есть прерывание будет вызываться
 * все время пока флаг установлен.
 *
 * **********************************************************************
 * Чтобы прерывания сработали они должны быть разрешены.
 * За разрешение прерываний отвечает регистр [INTERRUPT_ENABLE_REGISTER_2](IE2)
 *
 * флаг(бит) UCAxRXIE разрешает/запрещает прерывания при получении символа (когда выставлен флаг UCA0RXIFG)
 * флаг(бит) UCA0TXIE разрешает/запрещает прерывания при отправке символа (когда выставлен флаг UCA0TXIFG)
 *
 * PS При старте TXBUF свободен, соответсвенно флаг UCA0TXIFG выставлен и
 * как только мы установим флаг разрешения прерывания UCA0TXIE мы сразу же в это прерывание
 * влетим!
 *
 * **********************************************************************
 * RX - RECEIVE
 * TX - TRANSMIT
 ************************************************************************/

#define UART_RX_BUFFER  UCA0RXBUF //uart receive buffer
#define UART_TX_BUFFER  UCA0TXBUF //uart transmit buffer

// флаг выставляется когда символ приходит в буфер приемник (RXBUF)
#define UART_RX_FLAG_SET  (UCA0IFG & UCRXIFG)

// флаг выставляется
// 1) когда символ из буфера передачи (TXBUF) перебрасываются в сдвиговый регистр и TXBUF становится свободным для записи
// 2) after a PUC or when UCSWRST = 1
#define UART_TX_FLAG_SET  (UCA0IFG & UCTXIFG)

#define UART_RX_INTERRUPT_ENABLE()  (UCA0IE |= UCRXIE)
#define UART_TX_INTERRUPT_ENABLE()  (UCA0IE |= UCTXIE)
#define UART_TX_INTERRUPT_DISABLE() (UCA0IE &= ~UCTXIE)

/*------------ UART receive circular fifo buffer ------------*/
#define UART_RX_FIFO_BUFFER_SIZE 32
static uchar uart_rx_fifo_buffer[UART_RX_FIFO_BUFFER_SIZE];
static volatile uint uart_rx_buffer_head;
static volatile uint uart_rx_buffer_tail;
/*__________________________________________________*/

static uchar* uart_tx_data;
static volatile unsigned int uart_tx_data_size;

// TODO сделать enum с несколькими скоростями который передавать как параметр в init
void uart_init() {
    // DEFAULT: parity disabled - LSB - 8bit data - one stop bit - UART mode - Asynchoronous mode (page 577)
    // As said in 22.3.1 of the Holy User Guide,
    // before initialization of USCI we must reset it.
    UCA0CTL1 |= UCSWRST;          //stopping uart
    UCA0CTL1 |= UCSSEL_2;         //uart clock source - SMCLK
    //***setting the baud rate***
    UCA0BR1 = 0x00;
    UCA0BR0 = 0x02;

    UCA0MCTLW |= UCOS16;          //0th bit is set for high-speed clock source (SMCLK)
    UCA0MCTLW |= (3<<4);          //Second modulation stage
    UCA0MCTLW |= (0xAA<<8);       //First modulation stage
    //Setting pins (1.5-RX, 1.4-TX)
    P1REN &= ~(BIT5 + BIT4);
    P1DIR &= ~(BIT5);
    //P5DIR |= BIT4;
    P1SEL0 |= (BIT5 + BIT4);
    UCA0CTL1 &= ~UCSWRST;         //releasing uart  *Initialize USCI state machine*

    /*---------------- Interrupts ------------------*/
    UART_RX_INTERRUPT_ENABLE();
}

/**
* Не блокирующая  отправка  напрямую из переданного массива.
*  Переданный массив нельзя изменять пока все данные не будут отправлены.
* Также в это время нельзя добавлять другие данные на отправку
* Для работы по принципу двойной буфферизации
* (когда есть два массива одинаковой длины -
* один для отправку а второй в это время заполнять
*/
void uart_transmit(uchar* data, int data_size) {
    uart_tx_data = data;
    uart_tx_data_size = data_size;
    UCA0IFG |= UCTXIFG;         //Triggering Tx interrupt flag
    UART_TX_INTERRUPT_ENABLE();
}

/**
 * Блокирующая операция отправки данных. Для тестовых целей
 * Не делать одновременно блокирующее и неблокирующее отправление!!!
 * результат не предсказуем
 */
//void uart_blocking_transmit(uchar ch) {
//    // Wait for TX buffer to be ready for new data
//    while (! UART_TX_FLAG);
//    // Push data to TX buffer
//    UART_TX_BUFFER = ch;
//}

/**
 *  Waits for the transmission of outgoing uart data to complete
 */
void uart_flush() {
    while(uart_tx_data_size > 0);
}

void uart_rx_fifo_erase(){
    //for(int i = 0; i < UART_RX_FIFO_BUFFER_SIZE; i++){
    //    uart_rx_fifo_buffer[i] = 0;
    //}
    uart_rx_buffer_head=uart_rx_buffer_tail = 0;
}

/**
 * @return true если ассинхронная передача по UART завершены
 */
//bool uart_transmit_finished() {
//    if(uart_tx_data_size <= 0) {
//        return true;
//    }
//    return false;
//}

/**
 * Берет элемент из входящего fifo buffer где накапливаются поступающие данные
 * и записывает его в переменную по указанному адресу.
 * return true(1) if element was read successfully
 * and false(0) if uart fifo buffer is empty and element can not be read
 */
bool uart_read(uchar* chp) {
    // если буфер пустой
    if (uart_rx_buffer_head == uart_rx_buffer_tail) {
        return false;
    }
    *chp = uart_rx_fifo_buffer[uart_rx_buffer_tail];

    uint next_tail = (uint) (uart_rx_buffer_tail + 1);
    if (next_tail >= UART_RX_FIFO_BUFFER_SIZE) {
        next_tail = 0;
    }
    uart_rx_buffer_tail = next_tail;
    return true;
}

//***Combined UART Rx/Tx interrupt vector***
__attribute__((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR(void){
    uchar ch;
    uint next_head;
    switch(__even_in_range (UCA0IV, 18)){  //this intrinsic tells the compiler to omit odd checks, so the code is faster
        //Rx routine
        case 0x02:
            // Прочитать символ из буфера-приемника
            ch = UART_RX_BUFFER;
            // Проверить что uart fifo buffer не полон
            next_head = (uint) (uart_rx_buffer_head + 1);
            if (next_head >= UART_RX_FIFO_BUFFER_SIZE) {
                next_head = 0;
            }
            if (next_head != uart_rx_buffer_tail) { // буфер неполон
                // Положить пришедший символ в фифо буффер
                uart_rx_fifo_buffer[uart_rx_buffer_head] = ch;
                uart_rx_buffer_head = next_head;
            }
            break;
        //Tx routine
        case 0x04:
            if (uart_tx_data_size <= 0) { // Исходящий буфер пуст
                // Выключаем прерывание на передачу USCI
                UART_TX_INTERRUPT_DISABLE();
            } else {
                UART_TX_BUFFER = *uart_tx_data++;
                uart_tx_data_size--;
            }
            break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}


