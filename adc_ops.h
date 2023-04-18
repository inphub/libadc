/*
 * adc_ops.h
 *
 *  Created on: 7 April 2023
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include "adc.h"


void getCoefLine(double*yArr,double *xArr,unsigned int length,double *b,double *k);
void do_on_array(double *array,int length,void (*operation)(double *num));
double adc_ch_get_average(double *a_cells, unsigned a_cells_count, unsigned a_ch_id);

