/*
 * drs_cal_amp.h
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#pragma once
#include <dap_common.h>
#include "adc_cal.h"
#include "adc_cal_pvt.h"

#ifdef __cplusplus
extern "C" {
#endif

// Амплитудная калибровка
typedef struct {
    unsigned repeats; // (N) количество проходов амплитудной калибровки для каждого уровня цапов

    unsigned N; // (count) количество уровней у амплитудной калибровки levels_count,
                           // для каждого будет N (из repeats_count) проходов,
                           // при нуле будут выполняться два прохода для уровней BegServ и EndServ(о них ниже),
                           // при не нулевом значении, между  BegServ и EndServ будут включены count дополнительных уровней
                           // цапов для амплитудной калибровки
    double splash_gauntlet; // Уровень отсечения всплесков
    double levels[ADC_COUNT*ADC_CHANNELS_COUNT +2];
} adc_cal_amp_params_t;


void adc_calibrate_params_set_defaults(adc_cal_amp_params_t *a_params);

int adc_cal_amp( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);
void adc_cal_amp_remove_splash(drs_t * a_drs, double*a_Y, double a_gauntlet);


#ifdef __cplusplus
}
#endif
