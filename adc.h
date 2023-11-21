/*
 * adc.h
 *
 *  Создан: 26 сентября 2023
 *  Автор: Дмитрий Герасимов
 */
#pragma once

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>

#include <math.h>

#include <dap_common.h>
#include <dap_time.h>

#define ADC_COUNT 2

#define ADC_CHANNELS_COUNT 2
#define ADC_CHANNEL_BANK_COUNT 4
#define ADC_BANK_COUNT ADC_CHANNEL_BANK_COUNT * ADC_CHANNELS_COUNT

#define ADC_CELLS_COUNT_BANK      1024
#define ADC_CELLS_COUNT_CHANNEL   4*ADC_CELLS_COUNT_BANK
#define ADC_CELLS_COUNT           ADC_CHANNELS_COUNT*ADC_CELLS_COUNT_CHANNEL
#define ADC_CELLS_COUNT_ALL       ADC_COUNT*ADC_CELLS_COUNT


#define ADC_CELL_SIZE             sizeof(unsigned short)
#define ADC_PAGE_ALL_SIZE         (DRS_CELLS_COUNT_ALL * ADC_CELL_SIZE )
#define ADC_PAGE_COUNT_MAX              1024

// Задержка после чтения, в микросекундах
#define ADC_PAGE_READ_DELAY             50000

#define ADC_COEF_SPLASH           0x00000001
#define ADC_COEF_CHAN_K           0x00000002
#define ADC_COEF_CHAN_B           0x00000004
#define ADC_COEF_K                0x00000008
#define ADC_COEF_B                0x00000010

#define ADC_IDX(ch,n) ((n)*ADC_CHANNELS_COUNT+(ch))
#define ADC_IDX_BANK(ch,b,n) ((n + b*ADC_CELLS_COUNT_BANK)*ADC_CHANNELS_COUNT+(ch))

#define ADC_TOP_LEVEL 16384.0
#define ADC_VOLTAGE_BASE 0.5


typedef struct  {
  float offset;
  float gain;
} adc_params_ch_t;


typedef struct 
{
    adc_params_ch_t ch[ADC_CHANNELS_COUNT];
    dap_time_t init_on_start_timer_ms; // Не инициирует ничего, если 0
    void * _inheritor;
} adc_params_t;


typedef struct
{
    double b[ADC_CHANNELS_COUNT][ADC_CELLS_COUNT_CHANNEL];
    double k[ADC_CHANNELS_COUNT][ADC_CELLS_COUNT_CHANNEL];
    double chanB[ADC_CHANNELS_COUNT];
    double chanK[ADC_CHANNELS_COUNT];
    unsigned int splash[ADC_CHANNELS_COUNT];
    void * _inheritor;
} adc_coefficients_t;

/**
  ** @struct adc_t
  ** @details АЦП модуль
  */
typedef struct{
    short id;
    unsigned int shift_bank;
    unsigned int shift;

    adc_params_ch_t ch[ADC_CHANNELS_COUNT];

    adc_coefficients_t coeffs;

    void * _inheritor;
} adc_t;



// ---Флаги выставлять в том же порядке, что и перечислены ниже ---

// В аргументе перечисляет по очереди флаги АЦП модулей на инициализацию
#define ADC_INIT_ENABLE                    0x00000001

// Отрезает в начале массива указанное число ячеек при чтении
#define ADC_INIT_SET_DATA_CUT_FROM_BEGIN   0x00001000
//   Отрезает в конце массива указанное число ячеек при чтении
#define ADC_INIT_SET_DATA_CUT_FROM_END     0x00002000

// Верхний порог шкалы гейна ( в Дб)
#define ADC_GAIN_BEGIN  -6.0
// Нижний порог шкалы гейна (в Дб)
#define ADC_GAIN_END    26.0

#define ADC_GAIN_QUANTS_BEGIN    0
#define ADC_GAIN_QUANTS_END      32

extern double g_adc_current_freq; // Текущая частота дискретизации, в герцах
extern unsigned g_adc_data_cut_from_begin;
extern unsigned g_adc_data_cut_from_end;
extern adc_params_t * g_ini;
extern adc_t g_adc[ADC_COUNT];
extern int g_adc_flags;


#ifdef __cplusplus
extern "C" {
#endif

int adc_init(int a_adc_flags,...);
void adc_deinit();


void adc_reg_write(uint32_t a_reg_adr, uint32_t a_reg_data);
uint32_t adc_reg_read(uint32_t a_reg_adr);



#ifdef __cplusplus
}
#endif
