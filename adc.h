/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#pragma once

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>

#include <dap_common.h>
#include <dap_time.h>





#define ADC_COUNT 2

// Канал с калибровочной синусоидой для временной калибровки
#define ADC_CHANNEL_9 0

#define ADC_CHANNELS_COUNT 2
#define ADC_CHANNEL_BANK_COUNT 4
#define ADC_BANK_COUNT ADC_CHANNEL_BANK_COUNT * ADC_CHANNELS_COUNT

#define ADC_DATA_MAP_SIZE        0x4000

#define ADC_CELLS_COUNT_BANK      1024
#define ADC_CELLS_COUNT_CHANNEL   4*ADC_CELLS_COUNT_BANK
#define ADC_CELLS_COUNT           ADC_CHANNELS_COUNT*ADC_CELLS_COUNT_CHANNEL
#define ADC_CELLS_COUNT_ALL       ADC_COUNT*ADC_CELLS_COUNT

#define ADC_DCA_COUNT_ALL               4

#define ADC_DAC_COUNT (ADC_DCA_COUNT_ALL/ADC_COUNT)

#define ADC_CMD_INIT_SOFT_START			0x00000001 //EN_WORK
#define ADC_CMD_START_DRS			0x00000002
#define ADC_CMD_ENABLE_EXT_PULSE                0x00000004
#define ADC_CMD_RESET				0x00000008

#define ADC_REG_CMD_ADC_1		14
#define ADC_REG_CMD_ADC_2		15
#define ADC_MODE_REG			16

#define ADC_BASE_NUM_PAGE		19

#define ADC1_NUM_PAGE			19
#define ADC2_NUM_PAGE			20


#define ADC_REG_READY_A                 21
#define ADC_REG_READY_B                 22
#define ADC_REG_CALIB_SIN_ON_CH9        27

#define ADC_REG_DATA_DAC_CH9		31


#define ADC_REG_WAIT_ADC_A              49
#define ADC_REG_WAIT_ADC_B              50

#define ADC_PAGE_ALL_SIZE       (ADC_CELLS_COUNT_ALL * sizeof(unsigned short))

// Задержка после чтения, в микросекундах
#define ADC_PAGE_READ_DELAY             50000

#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2

#define MAX_PAGE_COUNT 1000

#define ADC_FAST_SIZE MAX_PAGE_COUNT*1024*8*8

#define ADC_ADC_TOP_LEVEL 16384.0
#define ADC_ADC_VOLTAGE_BASE 0.5

typedef struct  {
  float offset;
  float gain;
} adc_dac_ch_params_t;

typedef struct 
{
//  char firmware_path[256];
    uint32_t ROFS1;
    uint32_t OFS1;
    uint32_t ROFS2;
    uint32_t OFS2;
    uint32_t CLK_PHASE;
    float dac_offsets[4];
    float dac_gains[4];
    float adc_offsets[4];
    float adc_gains[4];
} DAP_ALIGN_PACKED fastadc_parameter_t;



typedef struct 
{
  fastadc_parameter_t fastadc;
  dap_time_t init_on_start_timer_ms; // Не инициирует ничего, если 0
} DAP_ALIGN_PACKED parameter_t;

typedef struct
{
    double b[ADC_CHANNELS_COUNT][ADC_CELLS_COUNT_CHANNEL];
    double k[ADC_CHANNELS_COUNT][ADC_CELLS_COUNT_CHANNEL];
    double b9[1*ADC_CELLS_COUNT_BANK];
    double k9[1*ADC_CELLS_COUNT_BANK];
    double kTime[1*ADC_CELLS_COUNT_BANK];
    double chanB[ADC_CHANNELS_COUNT];
    double chanK[ADC_CHANNELS_COUNT];
    double deltaTimeRef[ADC_CELLS_COUNT_BANK];
    unsigned int indicator;
    unsigned int splash[ADC_CHANNELS_COUNT];
} DAP_ALIGN_PACKED coefficients_t;

typedef struct{
    short id;
    unsigned int shift_bank;
    unsigned int shift;
    coefficients_t coeffs;
} adc_t;

typedef enum {
  ADC_MODE_SOFT_START = 0,
  ADC_MODE_EXT_START  = 1,
  ADC_MODE_PAGE_MODE  = 2,
  ADC_MODE_CAL_AMPL   = 3,
  ADC_MODE_CAL_TIME   = 4,
  ADC_MODE_OFF_INPUTS = 5,
  ADC_MODE_MAX = ADC_MODE_OFF_INPUTS
} adc_mode_t;

enum adc_freq{ADC_FREQ_1GHz,ADC_FREQ_2GHz,ADC_FREQ_3GHz,ADC_FREQ_4GHz,ADC_FREQ_5GHz};

extern enum adc_freq g_current_freq;

extern parameter_t * g_ini;
extern adc_dac_ch_params_t g_ini_ch9;

extern adc_t g_drs[ADC_COUNT];


#define SIZE_FAST MAX_PAGE_COUNT*1024*8*4*4

#define ADC_COEF_SPLASH           0x00000001
#define ADC_COEF_DELTA_TIME       0x00000002
#define ADC_COEF_CHAN_K           0x00000004
#define ADC_COEF_CHAN_B           0x00000008
#define ADC_COEF_K_TIME           0x00000010
#define ADC_COEF_K                0x00000020
#define ADC_COEF_B                0x00000040
#define ADC_COEF_K9               0x00000080
#define ADC_COEF_B9               0x00000100


#define ADC_IDX(ch,n) ((n)*ADC_CHANNELS_COUNT+(ch))
#define ADC_IDX_BANK(ch,b,n) ((n + b*ADC_CELLS_COUNT_BANK)*ADC_CHANNELS_COUNT+(ch))
#define ADC_IDX_CAL(n) ADC_IDX(ADC_CHANNEL_9,n)

#ifdef __cplusplus
extern "C" {
#endif

int adc_init();
int adc_cmd_init();
bool adc_get_inited();


void adc_deinit();
int adc_ini_load(const char *inifile, parameter_t *prm);


void adc_dac_shift_set_all(int a_adc_num, double *shiftDAC,float *DAC_gain,float *DAC_offset);
void adc_dac_shift_set_ch9(double a_shiftDAC,float DAC_gain,float DAC_offset);

void adc_dac_shift_input_set_all(int a_adc_num, unsigned short *shiftValue);

void adc_set_mode(int a_adc_num, adc_mode_t mode);

adc_mode_t adc_get_mode(int a_adc_num);

void adc_dac_shift_input_set(int a_adc_num, unsigned int value);
void adc_dac_shift_input_set_ch9(unsigned int a_value);

unsigned adc_dac_shift_input_get(int a_adc_num);
unsigned adc_dac_shift_input_get_ch9();


void adc_dac_set( unsigned int onAH);
void adc_set_freq(enum adc_freq a_freq);
double adc_get_freq_value(enum adc_freq a_freq);

void adc_reg_write(unsigned int reg_adr, unsigned int reg_data);
unsigned int adc_reg_read(unsigned int reg_adr);
void adc_start(int a_adc_num);
void adc_cmd(int a_adc_num, unsigned int a_cmd);

void adc_set_sinus_signal(bool a_sinus_signal);
unsigned int adc_get_shift(unsigned int a_adc_num);
unsigned int adc_get_shift_bank(unsigned int a_adc_num);
void adc_set_num_pages_all(unsigned int a_num);
void adc_set_num_pages(adc_t * a_drs, unsigned int a_num);
bool adc_get_flag_write_ready(int l_adc_num );
void adc_set_flag_end_read(int l_adc_num, bool a_enable);



extern void *data_map_drs1, *data_map_drs2, *data_map_shift_drs1, *data_map_shift_drs2, *data_map;


#ifdef __cplusplus
}
#endif
