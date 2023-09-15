/******************************************************************************
 * Copyright 2021 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "ist8310.h"
#include "mag.h"

#define DBG_TAG   "mag.ist8310"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define IST8310_ADDRESS  0x0E
#define REG_WHOAMI       0x00
#define REG_STAT1        0x02
#define REG_DATA_OUT_X_L 0x03
#define REG_DATA_OUT_X_H 0x04
#define REG_DATA_OUT_Y_L 0x05
#define REG_DATA_OUT_Y_H 0x06
#define REG_DATA_OUT_Z_L 0x07
#define REG_DATA_OUT_Z_H 0x08
#define REG_STAT2        0x09
#define REG_CTRL1        0x0a
#define REG_CTRL2        0x0b
#define REG_TEMP_L       0x1c
#define REG_TEMP_H       0x1d
#define REG_CTRL3        0x41
#define REG_CTRL4        0x42

#define CTRL1_ODR_SINGLE 0x01 /* Single measurement mode */
#define CTRL1_ODR_10_HZ  0x03
#define CTRL1_ODR_20_HZ  0x05
#define CTRL1_ODR_50_HZ  0x07
#define CTRL1_ODR_100_HZ 0x06
#define CTRL1_ODR_200_HZ 0x0B

#define CTRL2_SRST 0x01 /* Software reset */

#define CTRL3_SAMPLEAVG_16 0x24 /* Sample Averaging 16 */
#define CTRL3_SAMPLEAVG_8  0x1b /* Sample Averaging 8 */
#define CTRL3_SAMPLEAVG_4  0x12 /* Sample Averaging 4 */
#define CTRL3_SAMPLEAVG_2  0x09 /* Sample Averaging 2 */
#define CTRL4_SRPD         0xC0 /* Set Reset Pulse Duration */

#define IST8310_DEVICE_ID      0x10
#define IST8310_SCALE_TO_GAUSS 0.003f

#define IST8310_CONFIG_DEFAULT                              \
    {                                                       \
        100,                /* 100hz sample rate */         \
            50,             /* 16 times average */          \
            MAG_RANGE_16GA, /* xy +-16guess, z +-25guess */ \
    }

static struct rt_i2c_bus_device *i2c_bus;
static float _range_scale = IST8310_SCALE_TO_GAUSS;

__attribute__((weak)) void ist8310_user_calibrate(float data[3]);

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void ist8310_rotate_to_frd(float* data)
{
    /* do nothing */
}

static rt_err_t mag_raw_measure(int16_t mag[3])
{
    uint8_t buffer[6];

    i2c_read_regs(i2c_bus, IST8310_ADDRESS, REG_DATA_OUT_X_L, buffer, sizeof(buffer));

    /* swap the data */
    mag[0] = ((int16_t)buffer[1] << 8) | (int16_t)buffer[0];
    mag[1] = ((int16_t)buffer[3] << 8) | (int16_t)buffer[2];
    mag[2] = ((int16_t)buffer[5] << 8) | (int16_t)buffer[4];
    /* start next measurement */
    // i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL1, CTRL1_ODR_SINGLE));

    return RT_EOK;
}

static rt_err_t mag_measure(float mag[3])
{
    int16_t raw[3];

    mag_raw_measure(raw);

    mag[0] = _range_scale * raw[0];
    mag[1] = _range_scale * raw[1];
    mag[2] = _range_scale * raw[2];

    ist8310_rotate_to_frd(mag);

    if (ist8310_user_calibrate != RT_NULL) {
        /* do user defined calibration */
        ist8310_user_calibrate(mag);
    }

    return RT_EOK;
}

static rt_err_t probe(void)
{
    uint8_t device_id;

    i2c_read_reg(i2c_bus, IST8310_ADDRESS, REG_WHOAMI, &device_id);

    if (device_id != IST8310_DEVICE_ID) {
        LOG_E("ist8310 unmatched id: 0x%x\n", device_id);
        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t ist8310_init(void)
{
    /* check if device connected */
    probe();

    /* software reset */
    i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL2, CTRL2_SRST);

    rt_thread_mdelay(10);

    /* configure control register 3 */
    i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL3, CTRL3_SAMPLEAVG_16);

    /* configure control register 4 */
    i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL4, CTRL4_SRPD);

    /* Trigger one time measurement */
    // i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL1, CTRL1_ODR_SINGLE));

    i2c_write_reg(i2c_bus, IST8310_ADDRESS, REG_CTRL1, CTRL1_ODR_100_HZ);

    return RT_EOK;
}

/**
 * @brief 调用此函数读取 ist8310 数据
 *
 * @param data[3] 存储读取数据的数组
 *
 * @return 读取成功 RT_EOK ; 读取失败 -RT_ERROR
 */
static rt_err_t ist8310_read(float data[3])
{
    if (data == RT_NULL) {
        return -RT_ERROR;
    }

    if (mag_measure(data) != RT_EOK) {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief 调用此函数初始化 ist8310
 *
 * @param i2c_bus_name ist8310 所挂载的总线名称
 *
 * @return RT_EOK
 */
static rt_err_t drv_ist8310_init(const char* i2c_bus_name)
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(i2c_bus_name);
    RT_ASSERT(i2c_bus != NULL);

    ist8310_init();

    return RT_EOK;
}

struct mag_ops mag = {
    .mag_init = drv_ist8310_init,
    .mag_read = ist8310_read,
};
