/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @author  Wang Hongxi
 * @author  modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#include "drv_dwt.h"

#define DBG_TAG           "bsp.dwt"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static dwt_time_t systime;
static uint32_t cpu_freq_hz, cpu_freq_hz_ms, cpu_freq_hz_us;
static uint32_t CYCCNT_rount_count;
static uint32_t CYCCNT_last;
static uint64_t CYCCNT64;

/**
 * @brief 用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_rount_count
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static void dwt_cnt_update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT_CYCCNT;
        if (cnt_now < CYCCNT_last)
            CYCCNT_rount_count++;

        CYCCNT_last = DWT_CYCCNT;
        bit_locker = 0;
    }
}

void dwt_init(uint32_t cpu_freq_mhz)
{
    /* 使能DWT外设 */
    DEM_CR |= (uint32_t)DEM_CR_TRCENA;

    /* DWT CYCCNT寄存器计数清0 */
    DWT_CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;

    cpu_freq_hz = cpu_freq_mhz * 1000000;
    cpu_freq_hz_ms = cpu_freq_hz / 1000;
    cpu_freq_hz_us = cpu_freq_hz / 1000000;
    CYCCNT_rount_count = 0;

    dwt_cnt_update();
}

float dwt_get_delta(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT_CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(cpu_freq_hz));
    *cnt_last = cnt_now;

    dwt_cnt_update();

    return dt;
}

double dwt_get_delta_64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT_CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(cpu_freq_hz));
    *cnt_last = cnt_now;

    dwt_cnt_update();

    return dt;
}

void dwt_systime_update(void)
{
    volatile uint32_t cnt_now = DWT_CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    dwt_cnt_update();

    CYCCNT64 = (uint64_t)CYCCNT_rount_count * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / cpu_freq_hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * cpu_freq_hz;
    systime.s = CNT_TEMP1;
    systime.ms = CNT_TEMP2 / cpu_freq_hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - systime.ms * cpu_freq_hz_ms;
    systime.us = CNT_TEMP3 / cpu_freq_hz_us;
}

float dwt_get_time_s(void)
{
    dwt_systime_update();

    float DWT_Timelinef32 = systime.s + systime.ms * 0.001f + systime.us * 0.000001f;

    return DWT_Timelinef32;
}

float dwt_get_time_ms(void)
{
    dwt_systime_update();

    float DWT_Timelinef32 = systime.s * 1000 + systime.ms + systime.us * 0.001f;

    return DWT_Timelinef32;
}

uint64_t dwt_get_time_us(void)
{
    dwt_systime_update();

    uint64_t DWT_Timelinef32 = systime.s * 1000000 + systime.ms * 1000 + systime.us;

    return DWT_Timelinef32;
}

void dwt_delay_s(float delay)
{
    uint32_t tickstart = DWT_CYCCNT;
    float wait = delay;

    while ((DWT_CYCCNT - tickstart) < wait * (float)cpu_freq_hz);
}
