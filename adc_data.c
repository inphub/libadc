/*
 * adc_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#include <unistd.h>
#include <assert.h>
#include <dap_common.h>

#include "commands.h"
#include "drs.h"
#include "adc_data.h"
#include "adc_ops.h"

#define LOG_TAG "adc_data"

static bool s_debug_more=false;

/**
 * @brief adc_data_get_all
 * @param a_drs    ���� NULL �� �� �������� ��� ���� DRS
 * @param a_flags
 * @param a_buffer
 * @return
 */
int adc_data_get_all(adc_t * a_drs, int a_flags , unsigned short * a_buffer)
{
    if (a_drs){
        return adc_data_get(a_drs, a_flags, a_buffer, adc_CELLS_COUNT *sizeof(unsigned short) );
    }else for (size_t d=0; d < adc_COUNT; d++){
        int l_ret = adc_data_get(&g_drs[d],0,(unsigned short *) (((byte_t *) a_buffer) + adc_CELLS_COUNT *sizeof(unsigned short) ), adc_CELLS_COUNT *sizeof(unsigned short)  );
        if (l_ret!= 0){
            log_it(L_ERROR,"data not read on DRS #%u", d);
            return l_ret;
        }
    }
    return 0;
};


/**
 * @brief adc_data_get
 * @param a_drs                  ��������� �� ������ DRS
 * @param a_flags                ����� ��������
 * @param a_buffer               ����� ������, ������� adc_PAGE_READ_SIZE �������
 * @param a_buffer_size          ������������ ������ ������ ��� ������ (��� ������)
 */
int adc_data_get(adc_t * a_drs, int a_flags, unsigned short * a_buffer, size_t a_buffer_size )
{
    assert(a_drs);
    assert(a_buffer);
    unsigned int l_ret=0,i=0;
    unsigned l_cmds = adc_CMD_INIT_SOFT_START | adc_CMD_START_DRS;

    if (a_flags & adc_OP_FLAG_EXT_START){
        log_it(L_INFO, "start ext DRS");
        l_cmds |= adc_CMD_ENABLE_EXT_PULSE;
    }

    adc_cmd( -1, l_cmds);
    //usleep(100);

    bool l_is_ready = false;
    bool l_loop = true;
    while( l_loop ) {
        l_is_ready = adc_get_flag_write_ready(a_drs->id);
        if( l_is_ready)
            break;

        i++;
//        if( a_flags & adc_OP_FLAG_EXT_START){
            if(i>100){
                log_it(L_ERROR, "Was waiting for write_ready flag but without success");
                l_loop = false;
                l_ret = -1;
            }
//        }else{
            //if(ext_start==0){end=1;)
//        }
        //readExternalStatus(0xc); //Peter fix
    }
    if(l_is_ready ){
        debug_if(s_debug_more, L_DEBUG, "adc_data_get achieved on step #%u, DRS is %s", i, l_is_ready ? "ready" : "not ready");
    }else
        log_it(L_WARNING, "adc_data_get wasn't achieved after %u attempts, DRS is %s", i, l_is_ready ? "ready" : "not ready");

    if(a_flags & adc_OP_FLAG_ROTATE)
        adc_read_page_rotated(a_drs, 0, a_buffer, a_buffer_size);
    else
        adc_read_page(a_drs, 0, a_buffer, a_buffer_size);

    adc_set_flag_end_read(a_drs->id, true);

#ifndef adc_OPT_DATA_GET_NODELAYS
    usleep(adc_PAGE_READ_DELAY);
#endif

    return l_ret;
}


/**
 * @brief adc_read_page
 * @param a_drs
 * @param a_page_num
 * @param a_buffer
 * @param a_buffer_size
 */
void adc_read_page(adc_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    if ( a_drs->id ==0 )
        memcpy(a_buffer, (unsigned short *) ( ((byte_t*)data_map_drs1 )+ a_page_num*adc_PAGE_READ_SIZE), a_buffer_size ) ;
    else
       memcpy(a_buffer, (unsigned short *) ( ((byte_t*)data_map_drs2 )+ a_page_num*adc_PAGE_READ_SIZE), a_buffer_size ) ;
    a_drs->shift =adc_get_shift( a_drs->id);
    a_drs->shift_bank =a_drs->shift & 1023;

    //log_it(L_DEBUG, "Global shift: %u , local shift: %u",adc_get_shift(a_drs->id), a_drs->shift);

}


void adc_data_rotate(adc_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size)
{
    assert(a_drs);
    assert(a_mem_in);
    assert(a_mem_out);
    assert(a_cell_size);
    unsigned int l_shift_global = a_drs->shift;
    unsigned int l_shift = adc_CELLS_COUNT_BANK - a_drs->shift_bank;

    log_it(L_DEBUG, "Global shift: %u , local shift: %u",l_shift_global, l_shift);

    size_t l_total_size = 0;
    size_t l_in_offset, l_out_offset;

    const byte_t * l_buf_in = (byte_t*) a_mem_in;
    byte_t * l_buf_out =  DAP_NEW_STACK_SIZE(byte_t, a_mem_size);
    memset(l_buf_out,0, a_mem_size);

    // ������������� ������ ������
    for(unsigned b = 0; b < adc_CHANNEL_BANK_COUNT && l_total_size < a_mem_size; b++){
        size_t l_bank_shift = b*adc_CELLS_COUNT_BANK * adc_CHANNELS_COUNT *a_cell_size; // �������� �����
        size_t l_copy_size = (adc_CELLS_COUNT_BANK - l_shift) * a_cell_size * adc_CHANNELS_COUNT;

        if (l_copy_size + l_total_size > a_mem_size ) // ��������� �� ������� ������ �� ������� ������
            l_copy_size = a_mem_size - l_total_size;
        // �������� �������� �����
        l_out_offset = l_bank_shift;
        l_in_offset = l_bank_shift + l_shift*adc_CHANNELS_COUNT*a_cell_size;
        if(l_copy_size){
            memcpy( l_buf_out + l_out_offset,  l_buf_in + l_in_offset , l_copy_size ) ;
            l_total_size += l_copy_size;
        }

        // ���������, �� �� �� ���
        if(l_total_size >= a_mem_size)
            break;


        // ������� ������ ��������
        unsigned l_copy_size_tail = ( (adc_CELLS_COUNT_BANK * a_cell_size* adc_CHANNELS_COUNT) - l_copy_size);
        if ( (l_copy_size_tail + l_total_size) > a_mem_size ) // ��������� �� ������� ������ �� ������� ������
            l_copy_size_tail = a_mem_size - l_total_size;
        // �������� �������
        l_out_offset += l_copy_size ;
        l_in_offset = l_bank_shift;
        //memset( l_buf_out + l_out_offset, 0, l_copy_size_tail ) ;
        memcpy( l_buf_out + l_out_offset,
                l_buf_in + l_in_offset   , l_copy_size_tail ) ;
        l_total_size += l_copy_size_tail;

    }

    //memcpy(a_mem_out, l_buf_out, a_mem_size);
    // ������������� ��������� ��
    unsigned l_head_size = (adc_CELLS_COUNT_CHANNEL - l_shift_global) * a_cell_size * adc_CHANNELS_COUNT;
    memcpy(a_mem_out, l_buf_out + l_shift_global* a_cell_size * adc_CHANNELS_COUNT , l_head_size  );
    memcpy(((byte_t*)a_mem_out) + l_head_size, l_buf_out, l_shift_global * a_cell_size * adc_CHANNELS_COUNT );
}



/**
 * @brief adc_read_page_rotated
 * @param a_drs
 * @param a_page_num
 * @param a_buffer
 * @param a_buffer_size
 */
void adc_read_page_rotated(adc_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer,const size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    byte_t * a_adc_mem = a_drs->id == 0 ? (byte_t*)data_map_drs1 : (byte_t*)data_map_drs2;
    unsigned short * a_adc_page =(unsigned short *) (a_adc_mem + a_page_num*adc_PAGE_READ_SIZE);

    adc_data_rotate(a_drs, a_adc_page, a_buffer, a_buffer_size, sizeof(unsigned short));
}



/**
 * @brief adc_read_pages
 * @param a_drs
 * @param a_page_count
 * @param a_step
 * @param a_buffer
 * @param a_buffer_size
 */
void adc_read_pages(adc_t * a_drs, unsigned int a_page_count, unsigned int a_step,  unsigned short *a_buffer, size_t a_buffer_size)
{
    size_t l_offset = 0;
    bool l_loop = true;
    for (unsigned t=0; t< a_page_count && l_loop; t++){
        size_t l_read_size;
        if (l_offset + adc_PAGE_READ_SIZE <= a_buffer_size)
            l_read_size = adc_PAGE_READ_SIZE;
        else{
            l_read_size = a_buffer_size - l_offset;
            l_loop = false;
            log_it(L_ERROR, "Page read function goes out of input buffer, size %zd is not enought, requires %zd ( page read size %zd, num pages %u",
                    a_buffer_size, a_page_count * adc_PAGE_READ_SIZE, adc_PAGE_READ_SIZE, a_page_count );
        }
        adc_read_page(a_drs,t,&a_buffer[t*a_step], l_read_size);
    }
}



/**
 * unsigned int drsnum		����� drs ��� ����������� ������
 * return 					������ ������;
 */
unsigned int adc_get_shift_bank(unsigned int a_adc_num)
{
    return adc_get_shift(a_adc_num) &1023;
}

/**
 * @brief adc_get_shift
 * @param a_adc_num
 * @return
 */
unsigned int adc_get_shift(unsigned int a_adc_num)
{
    unsigned short tmpshift;
    if (a_adc_num==0)
     tmpshift=((unsigned long *)data_map_shift_drs1)[0];
    else
     tmpshift=((unsigned long *)data_map_shift_drs2)[0];

    return tmpshift;
}
