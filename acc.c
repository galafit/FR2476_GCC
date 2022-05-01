#include "msp430fr2476.h"
#include <stdbool.h>
#include "utypes.h"
#include "spi0.h"
#include "utils.h"
#include "interrupts.h"
#include "leds.h"

#define ACC_CS BIT1
#define ACC_SELECT() (P3OUT &= ~ACC_CS)
#define ACC_DESELECT() (P3OUT |= ACC_CS)

#define INT1 BIT2
#define INT2 BIT7

#define DATA_SIZE_INT 3                 //3 байта данные акселерометра
#define DATA_SIZE_LONG 3                 //3 байта данные акселерометра

static uchar data_size_char = 0;        //The total size of a data batch for accelerometers (in bytes)
static uchar data_size_int = 0;         //The total size of a data batch for accelerometers
static uchar acc_data_address[1] = {0};

/*
static uchar acc_reboot_int1[2] = {0x0D, 0x00};
static uchar acc_reboot_int2[2] = {0x0E, 0x00};
static uchar acc_reboot_acc[2] = {0x10, 0x00};
static uchar acc_reboot_gyr[2] = {0x11, 0x00};
static uchar acc_reboot_reset[2] = {0x12, 0x05};

static uchar acc_startup_int1[2] = {0x0D, 0x01};        //INT1 - ACC interrupt
static uchar acc_startup_int2[2] = {0x0E, 0x02};        //INT2 - GYR interrupt
static uchar acc_startup_acc[2] = {0x10, 0x70};         //ACC - 833hz
static uchar acc_startup_gyr[2] = {0x11, 0x72};         //GYR - 833hz
static uchar acc_startup_rounding[2] = {0x14, 0x60};    //Acc will automatically continue fetching gyr & acc data after reading the first register
static uchar acc_startup_bdu[2] = {0x12, 0x44};         //Prevents corruption of data when only one byte has been read (doesn't update data until both bytes are read)
static uchar acc_startup_drdy_mask[2] = {0x13, 0x08};   //Giving ACC Gyr time to settle (for low-pass filter)
//static uchar acc_data_address[1] = {0xA8};            //Address of the first data register of the "big" acc
*/

static uchar acc2_reset[2] = {0x20, 0x04};
static uchar acc2_who[2] = {0x8f, 0x00};
static uchar acc2_power[2] ={0x20 ,0xC3};               //Switch ACC on, BDU (prevent data corruption when reading just one byte
static uchar acc2_int1[2] = {0x21, 0x80};               //Включаем прерывание на пин INT1 при готовности данных
//static uchar acc2_int2[2] = {0x22, 0x80};             //Включаем прерывание на пин INT2 при готовности данных
static uchar acc2_SPI_speed[2] = {0x24, 1};             //Оптимизируем SPI на акселерометре для скоростей > 6Mhz


//Двойной буфер
static int double_buffer1[DATA_SIZE_INT];
static int double_buffer2[DATA_SIZE_INT];
static long accumulator1[DATA_SIZE_LONG];
static long accumulator2[DATA_SIZE_LONG];
static long* accumulators[2] = {accumulator1, accumulator2};
static long* current_accumulator = accumulator1;
static uchar ticker = 0;
static int* data_accumulator_fill = double_buffer1;
static int* data_accumulator_ready = double_buffer2;
//Будем считать сколько сэмплов сконвертировано за батч, чтобы потом делить
static unsigned long sample_counter1 = 1;
static unsigned long sample_counter2 = 1;
static unsigned long* sample_counters[2]= {&sample_counter1, &sample_counter2};
static unsigned long* current_sample_counter = &sample_counter1;
static uint data_prepared[DATA_SIZE_INT];
static uint data_display[DATA_SIZE_INT]; // actually triple buffer

static volatile bool acc_interrupt_flag;

static void acc_write_command(uchar* data, int data_size) {
    ACC_SELECT();
    while(data_size > 0){
        spi0_transfer(*data++);
        data_size--;
    }
    ACC_DESELECT();
}

// читает data_size байт в read_buffer начиная с адреса start_address
static void acc_read_registers(uchar start_address, uchar* read_buffer, int data_size) {
    ACC_SELECT();
    //Посылаем адрес первого регистра данных
    spi0_transfer(start_address);
    //Теперь остальные регистры прочитаются автоматически
    spi0_read((unsigned char *)read_buffer, data_size);
    ACC_DESELECT();
}

void acc_init(){
    spi0_init();
    //ACC пины: 2.2 - INT1, 2.7 - INT2, CS - 3.1
    //Инициализируем пин CS
    P3DIR |= ACC_CS;
    P3REN &= ~ACC_CS;
    //Стартуем акселерометр
    //Включаем питание
    wait(16000);
    P2REN &= ~BIT4;
    P2DIR |= BIT4;
    P2OUT |= BIT4;
    wait(16000);
    //Делаем Reset
    acc_write_command(acc2_reset, 2);
    wait(1000);
    //spi0_transmit(acc2_reset2, 2);
    //wait(1000);
    acc_write_command(acc2_SPI_speed, 2);
    acc_write_command(acc2_int1, 2);
    acc_write_command(acc2_power, 2);
    acc_write_command(acc2_who, 2);
    acc_write_command(acc2_who, 2);
    //Инициализируем пины для чтения прерываний от акселерометра
    P2SEL1 &= ~(INT1 + INT2);
    P2SEL0 &= ~(INT1 + INT2);
    P2REN &= ~(INT1 + INT2);
    P2IES &= ~(INT1 + INT2);        //При прерывании на пине, оно произойдет на восходящем фронте
    P2DIR &= ~(INT1 + INT2);
    data_size_char = 6;             //Пакет данных от акселерометра равен 6 байтам
    data_size_int = 3;
    acc_data_address[0] = 0xA8;     //Адрес первого регистра данных
}

void acc_stop_reading(){
    P2IE &= ~(INT1 /*+ INT2*/);         //Выключаем прерывание когда у акселерометра готовы данные
}

unsigned char* acc_get_data(){
    int i;
    //Фиксируем текущий буфер
    long* ready_accumulator = current_accumulator;
    unsigned long ready_sample_counter = *current_sample_counter;
    //Обнуляем счетчик сэмплов
    *current_sample_counter = 1;
    //Меняем двойной буфер
    current_accumulator = accumulators[(ticker++) & 1];
    current_sample_counter = sample_counters[ticker & 1];
    //Делаем данные беззнаковыми и делим на количество суммированных сэмплов
    for(i = 0; i < data_size_int; i++){
        //Количество сэмплов в пакете <= 23, но здесь ограничиваем 16ю
        data_prepared[i] = (unsigned int)((ready_accumulator[i] / 16/*/ ready_sample_counter*/) + 32768);
    }
    //Обнуляем аккумулятор
    for(int i = 0; i < DATA_SIZE_LONG; i++){
        ready_accumulator[i] = 0;
    }
    return (unsigned char*) data_prepared;
}

void acc_read(){
    P2IE |= (INT1 /*+ INT2*/);          //Включаем прерывание когда у акселерометра готовы данные
}


void acc_handle_interrupt() {
    if(acc_interrupt_flag) {
        acc_interrupt_flag = false;
        acc_read_registers(acc_data_address[0], (unsigned char *)double_buffer1, data_size_char);
        for(int i = 0; i < DATA_SIZE_LONG; i++){
            //Количество сэмплов в пакете <= 23, но здесь ограничиваем 16ю
            if(*current_sample_counter <= 16){
                current_accumulator[i] += (long)double_buffer1[i];
            }
        }
        *current_sample_counter += 1;
    }
}

__attribute__((interrupt(PORT2_VECTOR)))
void PORT2_ISR(void){
    switch(__even_in_range (P2IV, 0x10)){
    //INT1
    case 0x06:
        acc_interrupt_flag = true;
        break;
    //INT2
    case 0x10:
        break;
    }
    interrupt_flag = true;
    __low_power_mode_off_on_exit();
}

