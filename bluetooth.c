#include "msp430fr2476.h"
#include "interrupts.h"
#include "uart.h"
#include "utils.h"

#define BT_PROG BIT3
#define BT_RESET BIT0
#define BT_ON() (P4OUT &= ~BT_RESET)
#define BT_OFF() (P4OUT |= BT_RESET)
#define BT_PROG_ON() (P2OUT |= BT_PROG)
#define BT_PROG_OFF() (P2OUT &= ~BT_PROG)

unsigned char bt_name[] = "+NAME:BIOREC";               //Имя модуля, которое он должен нам послать, чтобы мы поняли, что он уже был запрограммирован
unsigned char ATname_enq[] = "AT+NAME?\r\n";            //Запрос имени
unsigned char ATname[] = "AT+NAME=BIOREC\r\n";          //Установка имени модуля
unsigned char ATreset[] = "at+reset\r\n";               //Перезагрузка модуля
unsigned char ATrole[] = "AT+ROLE=0\r\n";               //1 = Master, 0 = Slave
unsigned char ATuart[] = "AT+UART=460800,1,0\r\n";      //Настройки уарта
unsigned char ATaddr[] = "AT+ADDR?\r\n";                //Запрос адреса
unsigned char ATorgl[] = "AT+ORGL\r\n";                 //Factory reset
unsigned char input[32];                 //Буфер, куда мы будем писать все ответы модуля

unsigned char ifBtIsOk(){
  wait(600000);
  //Читаем ответ модуля и обнуляем массив с ответом, если ничего не получили
  int i = 0;
  while(uart_read(&input[i++]));
  if(i == 1){
      for(int j = 0; j < sizeof(input); j++){
          input[j] = 0;
      }
  }
  uart_rx_fifo_erase();
  //Проверяем, если модуль отвечает "ОК"
  //Если нет, то мы все команды будем писать заново
  if(input[0] == 'O'){
    if(input[1] == 'K'){
      return 0;
    }
  }
  return 1;     
}

void bt_program(){
  unsigned char isWriteSuccessful = 0;
  unsigned char errorCounter = 0;
  //Мы будем писать команды в модуль и проверять, если что отвечает "ОК"
  //Если нет, то мы все команды будем писать заново
  uart_rx_fifo_erase();
  while(isWriteSuccessful == 0){
    uart_transmit(ATorgl, (sizeof(ATorgl)-1));
    errorCounter += ifBtIsOk();
    uart_transmit(ATrole, (sizeof(ATrole)-1));
    errorCounter += ifBtIsOk();
    uart_transmit(ATname, (sizeof(ATname)-1));
    errorCounter += ifBtIsOk();
    uart_transmit(ATuart, (sizeof(ATuart)-1));
    errorCounter += ifBtIsOk();
    //Вроде эта команда не нужна
    //uart_transmit(ATaddr, (sizeof(ATaddr)-1));
    //errorCounter += ifBtIsOk();
    if(errorCounter > 0){
      isWriteSuccessful = 0;
      errorCounter = 0;
    }else{
      isWriteSuccessful = 1;
    }
  }
}

void bluetooth_init(){
  unsigned char UCA1BRW_prev;
  unsigned char UCA1MCTLW_prev;
  //Pin 2.3 - BT_PROG, 4.0 - BT_RESET
  P2OUT &= ~BT_PROG;
  P2SEL0 &= ~BT_PROG;
  P2SEL1 &= ~BT_PROG;
  P2REN &= ~BT_PROG;
  P2DIR |= BT_PROG;
  P4OUT |= BT_RESET;            //Reset activates at high level
  P4SEL1 &= ~BT_RESET;
  P4SEL0 &= ~BT_RESET;
  P4REN &= ~BT_RESET;
  P4DIR |= BT_RESET;
  //Запускаем модуль в режиме АТ команд
  //Сохраняем предыдущие настройки уарта и задаем скорость в 38400
  INTERRUPTS_DISABLE();
  UCA1CTLW0 |= UCSWRST;                   //Останавливаем уарт
  UCA1BRW_prev = UCA1BRW;
  UCA1BRW = 26;
  UCA1MCTLW_prev = UCA1MCTLW;
  UCA1MCTLW = 0x00;
  UCA1MCTLW |= (UCOS16 + (0xB6<<8) + (10<<4));
  UCA1STATW = 0x00;                    //Обнуляем регистр статума
  UCA1CTLW0 &= ~(UCSWRST);             //Запускаем уарт
  UCA1IE |= UCRXIE;                    //Устанавливаем прерывание на входящие данные
  INTERRUPTS_ENABLE();
  //Starting BT
  BT_PROG_OFF();
  BT_OFF();
  wait(1000000);
  BT_PROG_ON();                        //Включаем АТ режим на модуле
  wait(100);
  BT_ON();                             //Power on!
  wait(1000000);
  //Принудительный ресет всех параметров для теста прошивки блютуса
  //uart_transmit(ATorgl, (sizeof(ATorgl)-1));
  //wait(1000000);
  uart_rx_fifo_erase();
  uart_transmit(ATname_enq, (sizeof(ATname_enq)-1));          //Спрашиваем имя модуля
  wait(300000);
  //Если имя не такое, какое мы всем модулям задаем, то значит модуль новый и его надо запрограммировать
  int i = 0;
  //Читаем ответ модуля, ели ничего не получили, обнуляем входной массив
  while(uart_read(&input[i++]));
  if(i == 1){
      for(int j = 0; j < sizeof(input); j++){
          input[j] = 0;
      }
  }
  for(unsigned char i = 0; i < (sizeof(bt_name)-1); i++){
    //Если хоть один символ в имени не такой, как нам надо, то программируем модуль!
    if(input[i] != bt_name[i]){
      bt_program();
     break;
    }
  }
  //Восстанавливаем настройки уарта
  INTERRUPTS_DISABLE();
  UCA1CTLW0 |= UCSWRST;                //stopping uart
  UCA1BRW = UCA1BRW_prev;
  UCA1MCTLW = UCA1MCTLW_prev;
  UCA1STATW = 0x00;                    //resetting the status register
  UCA1CTLW0 &= ~(UCSWRST);             //releasing uart
  UCA1IE |= UCRXIE;                    //setting input interrupt
  INTERRUPTS_ENABLE();
  //Отключаем у модуля режим АТ команд и стартуем его в обычном режиме
  BT_OFF();
  BT_PROG_OFF();
  wait(200000);
  BT_ON();
  wait(1000000);
}

