/*
 * adc_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include <dap_common.h>
#include "adc.h"

// режим калибровки
#define ADC_OP_FLAG_CALIBRATE     BIT(0)
//индикатор внешнего запуска;
#define ADC_OP_FLAG_EXT_START     BIT(1)

#define ADC_OP_FLAG_ROTATE        BIT(3)

#define ADC_PAGE_READ_SIZE        ADC_CELLS_COUNT *sizeof(unsigned short)


#ifdef __cplusplus
extern "C" {
#endif

int adc_data_get(adc_t * a_drs, int a_flags, unsigned short * a_buffer, size_t  a_buffer_size);
int adc_data_get_all(adc_t * a_drs, int a_flags , unsigned short * a_buffer); /// Если a_drs NULL то он копирует для всех DRS



void adc_read_page(adc_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);
void adc_read_page_rotated(adc_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);
static inline void adc_read_page_all(adc_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer)
{
    return adc_read_page(a_drs, a_page_num, a_buffer, ADC_PAGE_READ_SIZE);
}

void adc_read_pages(adc_t * a_drs, unsigned int a_page_count, unsigned int a_step,  unsigned short *a_buffer, size_t a_buffer_size);

void adc_data_rotate(adc_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size);


#ifdef __cplusplus
}
#endif
