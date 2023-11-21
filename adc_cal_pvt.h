/*
 * adc_cal_pvt.h
 *
 *  Созданно: 26 Сентября 2023
 *  Автор: Дмитрий Герасимов <dmitry.gerasimov@demlabs.net>
 */

#pragma once
#include <dap_time.h>
#include <dap_string.h>
#include <stdatomic.h>

#include "adc_cal.h"

#define ADC_CAL_NUM_CYCLE_DEFAULT             1000
#define ADC_CAL_MAX_REPEATS_DEFAULT          10000
#define ADC_CAL_MIN_REPEATS_DEFAULT              2
#define ADC_CAL_SPLASH_GAUNTLET_DEFAULT      100.0
#define ADC_CAL_BEGIN_DEFAULT                -0.25
#define ADC_CAL_END_DEFAULT                   0.25
#define ADC_CAL_N_DEFAULT                      100
#define ADC_CAL_REPEATS                          1
#define ADC_CAL_MIN_N_DEFAULT                   50


typedef struct adc_calibrate{
    pthread_rwlock_t rwlock;

    bool is_running; // Запущен ли прямо сейчас

    atomic_uint_fast32_t progress; // Progress between 0 and 100
    unsigned progress_per_stage; // Количество прогресса на один тип калибровки

    pthread_t thread_id; // Айди потока
    adc_t * adc; // Объект DRS

    dap_nanotime_t ts_start;
    dap_nanotime_t ts_end;

    // Сигнализирует завершение калибровки
    pthread_cond_t  finished_cond;
    pthread_mutex_t finished_mutex;
} adc_calibrate_t;

// Аргументы калибровки
typedef struct adc_cal_args{
    union{
        struct{
            bool do_amplitude:1;
            bool do_time_local:1;
            bool do_time_global:1;
            unsigned padding:29;
        } DAP_ALIGN_PACKED;
        uint32_t raw; //  ключи калибровки, 1 бит амплитудная,2 локальная временная,3 глобальная временная
    } keys;
    adc_calibrate_params_t param;
    adc_calibrate_t * cal;
} adc_cal_args_t;

