#include "msp430fr2476.h"
#include <stdbool.h>
#include "ads1292.h"
#include "adc.h"
#include "acc.h"
#include "utypes.h"
#include "uart.h"
#include "leds.h"

#define START_MARKER 0xAA
#define STOP_MARKER 0x55

/**======================== Формат данных ======================
Универсальный упаковщик и для 2х канальной адс и для 8 канальной

Каждый пакет содержит 10 измерений от ADS
+ 1 измерение акселерометра по трем осям - X, Y, Z
+ 1 измерение батарейки

Каждый sample данных ADS занимает 3 байта.
Каждый sample данных от акселерометра занимает 2 байта
(то есть данные одного измерения от акселерометра по трем осям это 6 байт).

Данные имеют следующий вид:
 n_0 samples from ads_channel_0 (n_0 * 3 bytes)//if this ads channel enabled
 n_1 samples from ads_channel_1 (n_1 * 3 bytes)//if this ads channel enabled)
 ...
 n_8 samples from ads_channel_8 (n_8 * 3 bytes)  //if this ads channel enabled
 1 sample from accelerometer_x channel (2 bytes) //if accelerometer enabled
 1 sample from accelerometer_y channel (2 bytes) //if accelerometer enabled
 1 sample from accelerometer_Z channel (2 bytes) //if accelerometer enabled
 1 sample with BatteryVoltage info (2 bytes) //if BatteryVoltageMeasure  enabled
 1 byte(for 2 channels) or 2 bytes(for 8 channels) with lead-off detection info (if lead-off detection enabled) //сейчас пока этого нет

Количество самплов от ADS по каналу i:  n_i = 10/ divider_i
Последовательность байт в пакете Little Endian

Каждый пакет помимо данных содержит 2 стартовых байт в начале, стоповый байт в конце, а также номер пакета (счетчик пакетов)
и имеет следующий вид:
START_MARKER|START_MARKER|номер пакета(2bytes)|данные . . .|STOP_MARKER

 =========================================================**/


#define ADS_NUMBER_OF_CHANNELS 2
#define ADS_NUMBER_OF_MESURING 10 // 10 измерений на пакет
#define ADS_BYTES_PER_CHANNEL  (ADS_NUMBER_OF_MESURING * 3)
#define ADS_BATCH_SIZE (ADS_BYTES_PER_CHANNEL * ADS_NUMBER_OF_CHANNELS)  // 10 измерений, по 3 байта на каждый канал
#define ACC_ADC_DATA_SIZE 8 //4 канала по 2 байта каждый (3 канала акселерометра + батарейка)
#define BATCH_HEADER_SIZE 4 // start byte/start_byte/ batch_number (2 bytes)
#define BATCH_TAIL_SIZE 1 //stop byte

//Total size of the whole batch (10 samples for two channels+accelerometer,
// battery and a stop byte)
#define MAX_BATCH_SIZE (BATCH_HEADER_SIZE + ADS_BATCH_SIZE + ACC_ADC_DATA_SIZE + BATCH_TAIL_SIZE)

static int batch_size;
static uchar* ads_channel_dividers;

/*******  double buffer for all signals: ADS, ADC and helper info ******/
static uchar data_buffer_0[MAX_BATCH_SIZE];
static uchar data_buffer_1[MAX_BATCH_SIZE];
static uchar* fill_buffer = data_buffer_0; // ссылка на буфер для заполнения
static uchar* display_buffer = data_buffer_1;  //ссылка на заполненный буфер готовый для обработки
/***********************************************************************/
static bool acc_available = false;
static bool adc_available = false;

static bool is_recording = false;
//Counters for frames of data (batches)
static unsigned int batch_counter = 0;

//Pointers at ADS batch segments with offset for different channels
static unsigned char channel_pointers[ADS_NUMBER_OF_CHANNELS] = {0};
static unsigned char channel_starts[ADS_NUMBER_OF_CHANNELS];
static unsigned char ads_mesuring_count;


static void set_batch_size(){
    batch_size = BATCH_HEADER_SIZE + BATCH_TAIL_SIZE + ACC_ADC_DATA_SIZE;
    unsigned char channel;
    unsigned char channel_start = 0;
    unsigned char bytes_per_channel = 0;
    for(channel = 0; channel < ADS_NUMBER_OF_CHANNELS; channel++) {
        bytes_per_channel = ADS_BYTES_PER_CHANNEL / ads_channel_dividers[channel];
        batch_size += bytes_per_channel;
        channel_starts[channel] = channel_start;
        channel_pointers[channel] = channel_start;
        channel_start += bytes_per_channel;
    }
}

void databatch_init(bool adc_available1, bool acc_available1) {
    adc_available = adc_available1;
    acc_available = acc_available1;
    ads_init();
    if(adc_available) {
        adc_init();
        // передаем в ADS ссылку на функцию из ADC10 которую ads будет вызывать в прерывании DRDY при поступлении данных
        //ads_DRDY_interrupt_callback(adc_convert_begin);
    }
    if(acc_available) {
        acc_init();
    }
}

void databatch_start_recording(uchar* ads_dividers) {
    batch_counter = 0; //Setting the next batch number to zero
    ads_channel_dividers = ads_dividers;
    set_batch_size();
    ads_start_recording();
    if(adc_available) {
        adc_conversion_on(255);
    }
    if(acc_available) {
        acc_read();
    }
    batch_counter = 0;
    ads_mesuring_count = 0;
    is_recording = true;
}

void databatch_stop_recording() {
    ads_stop_recording();
    // consult with Stas ?
    if(adc_available) {
        adc_conversion_off();
    }
    if(acc_available) {
        acc_stop_reading();
    }
    is_recording = false;
}

/*
 * В пакет уже заполненный данными от 10 измерений ADS
 * добавляет данные от ACC и ADC (1 измерение), данные от батарейки (сейчас нули)
 * стартовые и стоповые байты и отправляет по UART
 */
static void make_batch(){
    //Adding data from accelerometer and adc
     //По 2 байта на каждую из осей x, y ,z в случае Accelerometer
     //По 2 байта на каждое измерение в случае ADC
    if(adc_available && acc_available) {
        uchar* adc_data = adc_get_data(); // adc data
        uchar* acc_data = acc_get_data(); // accelerometer data
        fill_buffer[batch_size - 9] = adc_data[0];
        fill_buffer[batch_size - 8] = adc_data[1];
        fill_buffer[batch_size - 7] = acc_data[0];
        fill_buffer[batch_size - 6] = acc_data[1];
        fill_buffer[batch_size - 5] = acc_data[2];
        fill_buffer[batch_size - 4] = acc_data[3];
    } else if(acc_available) {
        uchar* acc_data = acc_get_data();
        fill_buffer[batch_size - 9] = acc_data[0];
        fill_buffer[batch_size - 8] = acc_data[1];
        fill_buffer[batch_size - 7] = acc_data[2];
        fill_buffer[batch_size - 6] = acc_data[3];
        fill_buffer[batch_size - 5] = acc_data[4];
        fill_buffer[batch_size - 4] = acc_data[5];
    } else if (adc_available) {
        uchar* adc_data = adc_get_data();
        fill_buffer[batch_size - 9] = adc_data[0];
        fill_buffer[batch_size - 8] = adc_data[1];
        fill_buffer[batch_size - 7] = 0;
        fill_buffer[batch_size - 6] = 0;
        fill_buffer[batch_size - 5] = 0;
        fill_buffer[batch_size - 4] = 0;
    } else {
        fill_buffer[batch_size - 9] = 0;
         fill_buffer[batch_size - 8] = 0;
         fill_buffer[batch_size - 7] = 0;
         fill_buffer[batch_size - 6] = 0;
         fill_buffer[batch_size - 5] = 0;
         fill_buffer[batch_size - 4] = 0;
    }
    //Adding battery info
    //TODO think how to do it!!!
    fill_buffer[batch_size - 3] = 0;
    fill_buffer[batch_size - 2] = 0;
    //Stop marker
    fill_buffer[batch_size - 1] = STOP_MARKER;
    //Writing header info
    fill_buffer[0] = START_MARKER;
    fill_buffer[1] = START_MARKER;
    //Assigning  batch a number
    fill_buffer[2] = (uchar)batch_counter;
    fill_buffer[3] = (uchar)(batch_counter >> 8);
    //Increasing the batch no int (two bytes)
    batch_counter++;
    // swap double buffers
    uchar *tmp = display_buffer;
    display_buffer = fill_buffer;
    fill_buffer = tmp;
    uart_flush(); // ждем завершения отправки по uart
    if(is_recording) {
        //send data to uart
        uart_transmit(display_buffer, batch_size);
//        LED1_ON(); // дергаем пин P1.0 для запуска лог.анализатора
//        __delay_cycles(32);
//        LED1_OFF();
    }
}


/*
 * Метод помещает в пакет данные от 10 измерений ADS (ADS_NUMBER_OF_MESURING)
 */
#define LONG_MAX 2147483647
#define LONG_MIN 2147483648
static long accumulator[ADS_NUMBER_OF_CHANNELS] = {0};
static unsigned char accum_counter[ADS_NUMBER_OF_CHANNELS] = {0}; // все элементы массива будут инициализированы 0;
long avg_value;
long ads_value;
uchar* ptr_ads_value = &ads_value;
uchar* ptr_avg_value = &avg_value;
int i = 0;
long uart_value;
static void process_ads_samples(uchar* ads_samples){
    uchar* ads_buffer = fill_buffer + BATCH_HEADER_SIZE;
    signed char signed_byte;
    uchar channel;
    uchar chn_pointer;

    for(channel = 0; channel < ADS_NUMBER_OF_CHANNELS; channel++) {
        signed_byte = (signed char)*ads_samples; // старший байт определяет знак числа
        // MSP is little endian !!! MSP is little endian
        if(signed_byte < 0) {
            ptr_ads_value[3] = 0xFF;
        }else{
            ptr_ads_value[3] = 0;
        }

        ptr_ads_value[2] = *ads_samples++;
        ptr_ads_value[1] = *ads_samples++;
        ptr_ads_value[0] = *ads_samples++;

        //Accumulating for averaging.
        accumulator[channel] += ads_value;
        accum_counter[channel]++; //Keeping the count of accumulations
        //Stop accumulating when we have enough samples
        if(accum_counter[channel] >= ads_channel_dividers[channel]){
          //Averaging accumulated samples
            avg_value = accumulator[channel];
          if(ads_channel_dividers[channel] == 2) {
              avg_value = avg_value >> 1;     // делим на 2
          }else if(ads_channel_dividers[channel] ==  5) {
              avg_value = avg_value >> 3;     // делим на 8
          }else if(ads_channel_dividers[channel] == 10) {
              avg_value = avg_value >> 4; // делим на 16
          }

          accumulator[channel] = 0;
          accum_counter[channel] = 0;

          //Adding the result to the batch Порядок байт little_endian
          chn_pointer = channel_pointers[channel];

          ads_buffer[chn_pointer]     = ptr_avg_value[0];
          ads_buffer[chn_pointer + 1] = ptr_avg_value[1];
          ads_buffer[chn_pointer + 2] = ptr_avg_value[2];

          channel_pointers[channel] = chn_pointer + 3; //Pointing at the place to write the next sample
        }
    }

    // если ADS сделала все ADS_NUMBER_OF_MESURING (10) измерений то
    // завершаем формирование пакета и готовимся к формированию следующего
    if(++ads_mesuring_count >= ADS_NUMBER_OF_MESURING) {
        make_batch();
        ads_mesuring_count = 0;
        int channel;
         for(channel = 0; channel < ADS_NUMBER_OF_CHANNELS; channel++) {
             channel_pointers[channel] = channel_starts[channel];
         }
    }
}


void databatch_process() {
    if(ads_data_received()) {
        process_ads_samples(ads_get_data());
    }
}

