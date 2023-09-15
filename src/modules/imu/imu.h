/******************************************************************************
 * Copyright 2020-2023 The Firmament Authors. All Rights Reserved.
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

#ifndef GYRO_H__
#define GYRO_H__

#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_BMI088
#include "bmi088.h"
#endif /* BSP_USING_BMI088 */

/* ---------------------------------- GYRO ---------------------------------- */
/* gyro range type */
#define GYRO_RANGE_250DPS  250
#define GYRO_RANGE_500DPS  500
#define GYRO_RANGE_1000DPS 1000
#define GYRO_RANGE_2000DPS 2000

/* gyro device bus type */
#define GYRO_SPI_BUS_TYPE 1
#define GYRO_I2C_BUS_TYPE 2

/* default config for accel sensor */
#define GYRO_CONFIG_DEFAULT                              \
    {                                                    \
        2000,                   /* 2K sample rate */     \
            230,                /* 230Hz internal lpf */ \
            GYRO_RANGE_2000DPS, /* +-2000 deg/s */       \
    }

struct gyro_configure {
    rt_uint32_t sample_rate_hz;
    rt_uint16_t dlpf_freq_hz;
    rt_uint32_t gyro_range_dps;
};
/* ---------------------------------- GYRO ---------------------------------- */

/* ---------------------------------- ACCLE --------------------------------- */
/* accel range type */
#define ACCEL_RANGE_2G  2
#define ACCEL_RANGE_4G  4
#define ACCEL_RANGE_8G  8
#define ACCEL_RANGE_16G 16

/* accel device bus type */
#define ACCEL_SPI_BUS_TYPE 1
#define ACCEL_I2C_BUS_TYPE 2

/* default config for accel sensor */
#define ACCEL_CONFIG_DEFAULT                                    \
    {                                                           \
        800,                 /* 800 sample rate */              \
            280,             /* Normal BW */                    \
            ACCEL_RANGE_6G,  /* +-6g */                         \
    }

struct accel_configure {
    rt_uint32_t sample_rate_hz; /* sample rate in Hz */
    rt_uint16_t dlpf_freq_hz;   /* internal low-pass filter cur-off freq in Hz */
    rt_uint32_t acc_range_g;    /* accel measure range in g */
};
/* ---------------------------------- ACCLE --------------------------------- */

struct imu_ops{
    rt_err_t (*imu_init)(void);
    rt_err_t (*gyro_read)(float data[3]);
    rt_err_t (*gyro_config)(struct gyro_configure cfg);
    rt_err_t (*accel_read)(float data[3]);
    rt_err_t (*accel_config)(struct accel_configure cfg);
    float (*temp_read)(void);
};

#ifdef __cplusplus
}
#endif

#endif
