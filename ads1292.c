#include "spi1.h"
#include "msp430fr2476.h"
#include <stdbool.h>
#include "bynary.h"
#include "utypes.h"
#include "uart.h"
#include "leds.h"
// #include "ads1292.h"  // !!!! Посмотреть на стандартный ads1292.h  от TI
#include "interrupts.h"

/**
 * ADS выставляет флаг(бит) DRDY (data ready) в регистре флагов процессора, когда данные готовы.
 * Если разрешены прерывания по этому флагу то произойдет прерывание.
 * !!! Разобраться/определиться, что должно быть посля RESET !!!
 *
 * ADS прерывания привязаны к порту [P3]
 * [P3_INTERRUPT_FLAG_REGISTER] (P3IFG) порт где выставляется флаг прерываний DRDY
 * [P3_INTERRUPT_ENABLE_REGISTER] (P3IE) порт для разрешения/запрещения прерываний по DRDY
 *
 * Согласно datasheet все флаги PxIFG должны сбрасываться программно (?)
 * Поэтому в обработчике прерываний флаг порта (конкретно у нас выбран P3) DRDY нужно обязательно очищать вручную!
 * !!! Не путать с сигналом DRDY который (конкретно у нас приходит на BIT7 порта P3)
 */

/** Все определения относятся к P3 ****/
#define DRDY_BIT  BIT7
#define CS_BIT  BIT1
#define RESET_BIT  BIT2
#define ADS_DRDY_FLAG_SET  (P3IFG & DRDY_BIT)
#define ADS_DRDY_FLAG_CLEAR()  (P3IFG &= ~DRDY_BIT)
#define ADS_DRDY_INTERRUPT_ENABLE()  (P3IE |= DRDY_BIT)
#define ADS_DRDY_INTERRUPT_DISABLE()  (P3IE &= ~DRDY_BIT)
/***************************************/

#define NULL 0
#define ADC_NUMBER_OF_CHANNELS 2
#define ADS_SAMPLE_SIZE 9 // одно измерение: 3 байта служебные + 3 байта канал 1 + 3 байта канал 2  !!!!####!!! Выяснить про "3 байта служебные"
static uchar data_buffer[ADS_SAMPLE_SIZE]; //buffer for ads data
static uchar* display_buffer = data_buffer;

static volatile bool data_ready; // Сработало прерывание по сигналу DRDY от ADS на P3.7
static bool data_received;  // Dannye byli shitany po SPI

// Заготовки для задержек   Проверить, что берутся из msp430fr2476.h
#define DELAY_32()   __delay_cycles(32)
#define DELAY_64()   __delay_cycles(64)
#define DELAY_320()   __delay_cycles(320)
#define DELAY_450000()   __delay_cycles(450000)

#define SELECT_ADS() (P4OUT &= ~CS_BIT)   //Selecting ADS  // ??? Конкретно здесь можно просто закоротить PIN CS ADS ???
#define DESELECT_ADS() (P4OUT |= CS_BIT)   //deselecting ADS // Сейчас не используется.

// указатель на функцию без параметров например "void func()" определяется следующим образом: void (*func)(void)
// и дальше этому указателю можно присваивать адрес любой  соответсвующей функции и вызывать ее просто как func();
/** указатель на внешнюю функцию которая будет вызываться из прерывания DRDY (ads данные готовы) */
static void (*DRDY_interrupt_callback)(void); // сейчас не используется


/******** ADS ONE BYTE COMMANDS ( Набор команд opcode commands from data sheet: Table 15. Command Definitions Page 47) *********/
typedef enum {
    ADS_WAKEUP = B00000010, //Any following command must be sent after 4 tCLK cycles.
    ADS_STANDBY = B00000100, //It takes 9 fMOD cycles to execute
    ADS_RESET = B00000110,
    ADS_START = B00001000, //Start or restart (synchronize) conversions
    ADS_STOP = B00001010, //Stop conversion
    ADS_OFFSETCAL = B00011010, //Channel offset calibration
    ADS_ENABLE_CONTINUOUS_MODE = B00010000, //(RDATAC) Read Data Continuous
    ADS_DISABLE_CONTINUOUS_MODE = B00010001 //(SDATAC) Stop Read Data Continuously
} ADS_COMMAND;


//ADS initial register values for the proper startup (for testing purposes) ######## Надо разобраться !!! Посмотреть по DataShit
static uchar test_reg_values[] = {0x02, //reg 0x01 Set sampling ratio to 500 SPS
                                  0xA3,  //reg 0x02 Set internal reference PDB_REFBUF = 1, test enable
                                  0x10,  //reg 0x03
                                  0x10,  //reg 0x04 Set Channel 1 to input and set amplification to 1 (0x05  set channel  to test)
                                  0x10,  //reg 0x05 Route Channel 2 to input and set amplification to 1
                                  0x00,  //reg 0x06 Turn on Drl. //0x20 (Sasha) ?
                                  0x00,  //reg 0x07
                                  0x40,  //reg 0x08 clock divider Fclc/16 2048mHz external clock
                                  0xDE,  //Setting for respiratory circuit //0x02, //reg 0x09 Set mandatory bit. RLD REF INT doesn't work without it.
                                  0x07}; //Respiratory freq 64Khz//0x03}; //reg 0x0A Set RLDREF_INT


static void ads_write_command(ADS_COMMAND command) {
    DELAY_32();
    spi1_transfer(command);  // Этот фаил N = 52 - 61
    DELAY_32();
}

/**
 * Запись подряд нескольких регистров
 * @param addres - starting register address
 * @param data указатель на массив данных
 * @param data_size размер данных
 */
void ads_write_regs(uchar address, uchar* data, uchar data_size) {
    DELAY_32();
    //The Register Write command is a two-byte opcode followed by the input of the register data.
    //First opcode byte: 010r rrrr, where r rrrr is the starting register address.
    //Second opcode byte: 000n nnnn, where n nnnn is the (number of registers to write � 1)
    uchar opcode_first_byte = address | B01000000;
    uchar opcode_second_byte = data_size - 1; // (number of registers to write � 1)
    // отправляем команду записи в регистр
    spi1_transfer(opcode_first_byte);
    spi1_transfer(opcode_second_byte);
    for (uchar i = 0; i < data_size; i++) {
        spi1_transfer(data[i]);  // write data
    }
    DELAY_32();
}

//initial ADS startup for testing purposes
static void ads_test_config() {
    ads_write_command(ADS_DISABLE_CONTINUOUS_MODE);   //Disable Read Data Continuous mode
    ads_write_command(ADS_STOP);  //Stop recording, the ADS will be "auto" deselected
    //writing test startup data
    ads_write_regs(0x01, test_reg_values, sizeof(test_reg_values));
}

void ads_init() {  // !!!! Надо все перепроверить
    spi1_init();
    //Configuring ports
    //4.1=CS, 4.2=RESET, 3.7=DRDY
    P4DIR |= (CS_BIT + RESET_BIT);
    P4SEL0 &= ~(CS_BIT + RESET_BIT);
    P4SEL1 &= ~(CS_BIT+ RESET_BIT);
    //Ads in reset state and stopped
    P4OUT &= ~(RESET_BIT); //  may be it is not needed. Read datasheet?
    //DRDY pin is input, sensitive to high-to-low transition
    P3SEL1 &= ~DRDY_BIT;  // P3.7 DRDY
    P3SEL0 &= ~DRDY_BIT;
    P3REN &= ~DRDY_BIT;
    P3IES |= DRDY_BIT;  //When interrupt is on, it will happen on high-to-low edge transition
    P3DIR &= ~DRDY_BIT;
    //Startup,
    //Releasing ADS, wait for it to start and then reset
    SELECT_ADS();
    DELAY_32();
    P4OUT |= RESET_BIT;
    DELAY_450000();
    P4OUT &= ~RESET_BIT;  // ads reset
    DELAY_64();
    P4OUT |= RESET_BIT; // ads releasing
    DELAY_320();
  //  ads_test_config();
}

/**
 * Чтение одного регистра. Возращает прочитанное значение
 * (Не используем, но есть в файле commands.c  Резерв для отладки)
 */
uchar ads_read_reg(uchar address) {
    DELAY_32();
    //The Register Read command is a two-byte opcode followed by the output of the register data.
    //First opcode byte: 001r rrrr, where r rrrr is the starting register address.
    //Second opcode byte: 000n nnnn, where n nnnn is the number of registers to read � 1.
    uchar opcode_first_byte = address | B00100000;
    uchar opcode_second_byte = 0x00; // (number of registers to read � 1) = 0
    // отправляем команду чтения из регистра
    spi1_transfer(opcode_first_byte);
    spi1_transfer(opcode_second_byte);
    // отправляем 0 чтобы прочитать данные
    uchar reg_value;
    spi1_read(&reg_value, 1); //Reading one byte
    return reg_value;
}

void ads_stop_recording() { // Разобраться, что происходит. Почему дергается сигнал DRDY
    LED1_OFF();
    ads_write_command(ADS_DISABLE_CONTINUOUS_MODE); // stop continuous recording
    ads_write_command(ADS_STOP); //ads stop
}

void ads_start_recording() {
    LED1_ON();
    ADS_DRDY_INTERRUPT_DISABLE(); //disable interrupt on DRDY чтобы прерывания не нарушали процесс старта
    // очищаем флаги
    ADS_DRDY_FLAG_CLEAR(); //Clearing interrput flag DRDY
    data_ready = false;
    ads_write_command(ADS_ENABLE_CONTINUOUS_MODE); // enable continuous recording
    ads_write_command(ADS_START); //start recording
    ADS_DRDY_INTERRUPT_ENABLE(); //Enabling the interrupt on DRDY
}

bool ads_data_received() {
    if (data_ready) {
        /****** Обработчик прерывания *****/
        //чтение данных из ADS по SPI
        spi1_read(data_buffer, ADS_SAMPLE_SIZE);
        // вызвываем callback функцию если ее адрес не нулевой
        if (*DRDY_interrupt_callback != NULL) {
            DRDY_interrupt_callback();
        }
        data_ready = false;
        data_received = true;
        /*************************************/
    }
    return data_received;
}

/**
 * Перед тем как получить данные убедиться что они готовы. Метод ads_data_received()!
 *
 * Возвращает ссылку на массив из 3 * ADS_NUMBER_OF_CHANNELS  байт:
 * 3 байта от первого канала
 * 3 байта от второго канала
 * ...
 * Порядок байт BIG ENDIAN
 */
uchar* ads_get_data() {
    data_received = false;
    //Dropping the first 3 bytes from ADS (там служебная информация)
    return data_buffer + 3;
}

// отправляет тестовые данные
uchar test_data[6] = { 0xA9, 0x06, 0x60, 0xA9, 0x06, 0x60};
uchar* ads_get_data_t() {
    data_received = false;
    return test_data;
}

/**
 * Перед тем как получить значение лофф статуса
 * убедиться что данные от ADS считаны. Метод ads_data_received()
 */
// скопировано у Саши. Разобраться что за 3 служебных байта выдает ADS
/*uchar ads_get_loff_status() {
    uchar result = ((data_buffer[0] << 1) & 0x0E) | ((data_buffer[1] >> 7) & 0x01);
    return result;
}*/

// метод передает указатель на конкретную функцию которая будет вызываться в DRDY прерывании (данные готовы)
/*void ads_DRDY_interrupt_callback(void (*func)(void)) {
    DRDY_interrupt_callback = func;
}*/


__attribute__((interrupt(PORT3_VECTOR)))
void PORT3_ISR(void){
    if (ADS_DRDY_FLAG_SET) { //if interrupt from DRDY
        data_ready = true; // выставляем флаг
        ADS_DRDY_FLAG_CLEAR();
//        LED1_ON(); // дергаем пин P1.0 для запуска лог.анализатора
//        __delay_cycles(32);
//        LED1_OFF();
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit(); // Выходим из спячки в main loop
}










