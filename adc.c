/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#include <assert.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include <pthread.h>


#include <dap_common.h>
#include <dap_config.h>
#include <dap_file_utils.h>
#include <dap_timerfd.h>

#define LOG_TAG          "adc"

#include "adc.h"
#include "adc_ops.h"
#include "adc_cal.h"
#include "adc_cli.h"
#include "adc_math.h"


adc_params_t * g_ini = NULL;

adc_t g_adc[ADC_COUNT]={};
double g_adc_current_freq = 0.0; // Текущая частота дискретизации, в герцах
void *s_data_map = NULL;
byte_t * s_mem_reg = NULL
int g_adc_flags = 0;
va_list s_adc_flags_vars;

unsigned g_adc_data_cut_from_begin = 0;
unsigned g_adc_data_cut_from_end = 0;
unsigned short g_adc_gain_default = ADC_GAIN_QUANTS_END;

static int s_mem_fd = 0;
static char s_adc_check_file[]="/tmp/adc_init";

static bool s_debug_more = false;
static bool s_initalized = false;
static pthread_cond_t s_initalized_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t s_initalized_mutex = PTHREAD_MUTEX_INITIALIZER;


static bool s_init_on_start_timer_callback(void* a_arg); // Init on start timeout callback
static int s_init_mem(void);
static void s_hw_init();
static int s_post_init();

static void s_dac_set(unsigned int onAH);
static int s_ini_load(const char *a_ini_path, adc_params_t *a_prm);

/**
 * @brief dap_get_inited
 * @return
 */
static inline bool s_get_inited()
{
    return false;
    if (dap_config_get_item_bool_default(g_config,"common","check_hw_init", false))
        return dap_file_test(s_adc_check_file);
    else
        return false;
}


/**
 * @brief adc_init
 * @param a_adc_flags
 * @return
 */
int adc_init(int a_adc_flags,...)
{
    va_list a_vars;
    // Проверка на инициализацию
    if( s_initalized ) {
        log_it(L_WARNING, "ADC is already initialized, pls check your code for double call");
        return -1000;
    }
    s_initalized = true;
    g_adc_flags = a_adc_flags;

    for (unsigned i = 0; i < ADC_COUNT; i++)
      g_adc[i].id = i;

    va_start(a_vars, a_adc_flags);
    va_copy(s_adc_flags_vars, a_vars);
    va_end(a_vars);

    // Инициализация ADC
    s_init_mem();

    g_ini = DAP_NEW_Z(adc_params_t);
    s_ini_load("/media/card/config.ini", g_ini );

    if(g_adc_flags & ADC_INIT_SET_DATA_CUT_FROM_BEGIN ){
        g_adc_data_cut_from_begin = va_arg(s_adc_flags_vars, unsigned);
    }

    if(g_adc_flags & ADC_INIT_SET_DATA_CUT_FROM_END )
        g_adc_data_cut_from_end = va_arg(s_adc_flags_vars, unsigned);


    // Инициализация параметров DRS по таймеру

    log_it(L_NOTICE,"ADC config and memory are initialized");

    pthread_mutex_lock(&s_initalized_mutex);

    dap_timerfd_start_on_worker(  dap_events_worker_get_auto(),  g_ini->init_on_start_timer_ms,
                                   s_init_on_start_timer_callback, g_ini);

    pthread_cond_wait(&s_initalized_cond, &s_initalized_mutex);
    pthread_mutex_unlock(&s_initalized_mutex);

    return 0;
}

/**
 * @brief s_init_mem
 * @return
 */
static int s_init_mem(void)
{
    int ret = EXIT_FAILURE;
//	unsigned char value;


    /* open the memory device file */
    s_mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (s_mem_fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    /* map the LWHPS2FPGA bridge into process memory */
/*
 *     off_t control_base = LWHPS2FPGA_BRIDGE_BASE;
 *     control_map = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, s_mem_fd, control_base);
    if (control_map == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }*/

cleanup:
    return ret;
}

/**
 * @brief s_deinit_mem
 */
static void s_deinit_mem(void)
{

cleanup:
    close(s_mem_fd);
}

/**
 * @brief s_init_on_start_timer_callback
 * @details Callback for timer. If return true,
 *          it will be called after next timeout
 * @param a_arg
 * @return
 */
static bool s_init_on_start_timer_callback(void* a_arg)
{
    UNUSED(a_arg);
    if( ! s_get_inited() ){
        log_it(L_INFO,"Timeout for init on start passed, initializing ADC...");
        s_hw_init();
    }else{
        log_it(L_DEBUG,"ADC is already initialized so lets just pass this stage");
    }
    s_post_init();

    // Чтобы точно успел ожидающий поток встать на pthread_cond_wait
    pthread_mutex_lock(&s_initalized_mutex);
    pthread_mutex_unlock(&s_initalized_mutex);

    // А вот тут собственно и вызываем
    pthread_cond_broadcast(&s_initalized_cond);


    return false;
}

/**
 * @brief s_post_init
 */
static int s_post_init()
{
    // Инициализация консоли
    if (drs_cli_init() != 0){
        log_it(L_CRITICAL, "Can't init adc cli");
        return -14;
    }
    // Инициализация калибровочных модулей
    drs_calibrate_init();

    return 0;
}

/**
 * @brief drs_init
 * @param a_drs_flags
 */
static void s_hw_init()
{

    if( s_get_inited() ){
        log_it(L_WARNING, "Already initialized");
        return;
    }

    /// TODO HW init callback

    // Touch file
    FILE * f = fopen(s_adc_check_file,"w");
    fclose(f);

    // Init all DRS
    //drs_cmd(-1, DRS_CMD_ );

    log_it(L_NOTICE, "ADC settings are implemented");

    /// TODO HW post init callback

}



/**
 * @brief adc_deinit
 */
void drs_deinit()
{
   adc_calibrate_deinit();

   DAP_DELETE(g_ini);
   g_ini = NULL;
}


/**
 * @brief Загружает даные из ini файла и сохраняет в параметры
 * @param a_ini_path
 * @param a_prm
 */
static int s_ini_load(const char *a_ini_path, adc_params_t *a_prm)
{
    unsigned char t;
    dap_config_t * l_cfg = dap_config_load(a_ini_path);
    if ( l_cfg == NULL){
        log_it(L_CRITICAL, "Can't load ini file from path %s", a_ini_path);
        return -1 ;
    }

    a_prm->init_on_start_timer_ms                  = dap_config_get_item_uint32_default(l_cfg,"common","init_on_start_timer",1000);

    dap_config_close( l_cfg );
    return 0;
}



/**
 * double *shiftDAC		;
 */

void adc_reg_write(uint32_t a_reg_adr, uint32_t a_reg_data)
{
    /* get the delay_ctrl peripheral's base address */
    uint32_t * l_mem = (uint32_t *) ( (s_mem_reg + a_reg_adr*4);
    debug_if(s_debug_more, L_DEBUG, "write: adr=0x%08x (%u), val=0x%08x", a_reg_adr, a_reg_adr, a_reg_data);

    /* write the value */
    *l_mem = a_reg_data;
    usleep(100);
}

/**
 * @brief adc_reg_read
 * @param a_reg_adr
 * @return
 */
unsigned int adc_reg_read(unsigned int a_reg_adr)
{
    register uint32_t l_reg_data;
    uint32_t * l_mem = (uint32_t *) (s_mem_reg + a_reg_adr*4);
    usleep(100);
    debug_if(s_debug_more, L_DEBUG, "read: adr=0x%08x (%u), val=0x%08x", a_reg_adr, a_reg_adr, *l_mem);
    return  *l_mem;
}

