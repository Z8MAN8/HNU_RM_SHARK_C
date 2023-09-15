/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
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
#include "bmi088.h"
#include "drv_dwt.h"
#include "rm_config.h"
#include <drv_spi.h>
#include <math.h>

#define DBG_TAG   "imu.bmi088"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#ifdef BIT
    #undef BIT
#endif

#define BIT(_idx) (1 << _idx)
#define REG_VAL(_setbits, _clearbits) \
    (reg_val_t) { .setbits = (_setbits), .clearbits = (_clearbits) }

// #define BMI088_ACC_I2C_ADDR1        0x18                 //SDO is low(GND)
// #define BMI088_ACC_I2C_ADDR2        0x19                 //SDO is high(VCC)
// #define BMI088_ACC_DEFAULT_ADDRESS  BMI088_ACC_I2C_ADDR2 //in the LPC54102 SPM-S

#define BMI088_ACC_BGW_CHIPID_VALUE 0x1E
#define BMI088_ACC_BGW_CHIPID       0x00

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACC_STATUS  0x03

#define BMI088_ACCD_X_LSB   0x12
#define BMI088_ACCD_X_MSB   0x13
#define BMI088_ACCD_Y_LSB   0x14
#define BMI088_ACCD_Y_MSB   0x15
#define BMI088_ACCD_Z_LSB   0x16
#define BMI088_ACCD_Z_MSB   0x17
#define BMI088_SENSORTIME_0 0x18
#define BMI088_SENSORTIME_1 0x19
#define BMI088_SENSORTIME_2 0x1A

#define BMI088_INT_STAT_1 0x1D

#define BMI088_INT_TEMP_MSB 0x22
#define BMI088_INT_TEMP_LSB 0x23

#define BMI088_ACC_CONF  0x40
#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_PWR_CONF  0x7C
#define BMI088_ACC_PWR_CTRL  0x7D
#define BMI088_ACC_SOFTRESET 0x7E

// #define BMI088_GYRO_I2C_ADDR1        0x68 //SDO is low(GND)
// #define BMI088_GYRO_I2C_ADDR2        0x69 //SDO is high(VCC)
// #define BMI088_GYRO_DEFAULT_ADDRESS  BMI088_GYRO_I2C_ADDR2
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80

#define BMI088_GRRO_CHIP_ID 0x0F
#define BMI088_CHIP_ID_ADDR 0x00

#define BMI088_RATE_X_LSB_ADDR 0x02
#define BMI088_RATE_X_MSB_ADDR 0x03
#define BMI088_RATE_Y_LSB_ADDR 0x04
#define BMI088_RATE_Y_MSB_ADDR 0x05
#define BMI088_RATE_Z_LSB_ADDR 0x06
#define BMI088_RATE_Z_MSB_ADDR 0x07

#define BMI088_INTR_STAT1_ADDR 0x0A

#define BMI088_RANGE_ADDR 0x0F

#define BMI088_BW_ADDR 0x10

#define BMI088_MODE_LPM1_ADDR 0x11

#define BMI088_BGW_SOFT_RST_ADDR 0x14

#define BMI088_INTR_ENABLE0_ADDR 0x15
#define BMI088_INTR_ENABLE1_ADDR 0x16

#define BMI088_SELECTF_TEST_ADDR 0x3C

#define BMI088_GYRO_RANGE_2000_DPS REG_VAL(0, BIT(2) | BIT(1) | BIT(0))
#define BMI088_GYRO_RANGE_1000_DPS REG_VAL(BIT(0), BIT(2) | BIT(1))
#define BMI088_GYRO_RANGE_500_DPS  REG_VAL(BIT(1), BIT(2) | BIT(0))
#define BMI088_GYRO_RANGE_250_DPS  REG_VAL(BIT(1) | BIT(0), BIT(2))
#define BMI088_GYRO_RANGE_125_DPS  REG_VAL(BIT(2), BIT(1) | BIT(0))

#define BMI088_GYRO_RATE_100  REG_VAL(BIT(2) | BIT(1) | BIT(0), BIT(3))
#define BMI088_GYRO_RATE_200  REG_VAL(BIT(2) | BIT(1), BIT(3) | BIT(0))
#define BMI088_GYRO_RATE_400  REG_VAL(BIT(1) | BIT(0), BIT(3) | BIT(2))
#define BMI088_GYRO_RATE_1000 REG_VAL(BIT(1), BIT(3) | BIT(2) | BIT(0))
#define BMI088_GYRO_RATE_2000 REG_VAL(BIT(0), BIT(3) | BIT(2) | BIT(1))

#define BMI088_ACCEL_RANGE_3_G  0x00
#define BMI088_ACCEL_RANGE_6_G  0x01
#define BMI088_ACCEL_RANGE_12_G 0x02
#define BMI088_ACCEL_RANGE_24_G 0x03

#define BMI088_ACCEL_BW_12_5 0xA5
#define BMI088_ACCEL_BW_25   0xA6
#define BMI088_ACCEL_BW_50   0xA7
#define BMI088_ACCEL_BW_100  0xA8
#define BMI088_ACCEL_BW_200  0xA9
#define BMI088_ACCEL_BW_400  0xAA
#define BMI088_ACCEL_BW_800  0xAB
#define BMI088_ACCEL_BW_1600 0xAC

#define BMI088_ACCEL_RATE_12_5 REG_VAL(BIT(0) | BIT(2), BIT(1) | BIT(3))
#define BMI088_ACCEL_RATE_25   REG_VAL(BIT(1) | BIT(2), BIT(0) | BIT(3))
#define BMI088_ACCEL_RATE_50   REG_VAL(BIT(0) | BIT(1) | BIT(2), BIT(3))
#define BMI088_ACCEL_RATE_100  REG_VAL(BIT(3), BIT(0) | BIT(1) | BIT(2))
#define BMI088_ACCEL_RATE_200  REG_VAL(BIT(0) | BIT(3), BIT(1) | BIT(2))
#define BMI088_ACCEL_RATE_400  REG_VAL(BIT(1) | BIT(3), BIT(0) | BIT(2))
#define BMI088_ACCEL_RATE_800  REG_VAL(BIT(0) | BIT(1) | BIT(3), BIT(2))
#define BMI088_ACCEL_RATE_1600 REG_VAL(BIT(2) | BIT(3), BIT(0) | BIT(1))

#define BMI088_ACCEL_OSR_0 REG_VAL(BIT(5) | BIT(7), BIT(4) | BIT(6))
#define BMI088_ACCEL_OSR_2 REG_VAL(BIT(4) | BIT(7), BIT(5) | BIT(6))
#define BMI088_ACCEL_OSR_4 REG_VAL(BIT(7), BIT(4) | BIT(5) | BIT(6))

#define M_PI_F       3.1415926f
#define BMI088_ONE_G 9.80665f
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

typedef struct {
    uint8_t setbits;
    uint8_t clearbits;
} reg_val_t;

static rt_device_t gyro_spi_dev;
static rt_device_t accel_spi_dev;
static float gyro_range_scale;
static float accel_range_scale;
static float sample_rate;
/* 通过校准得到的数据 */
static float gyro_offset[3];
static uint8_t cali_count;  // 校准次数
float bmi088_g_norm;        // 通过校准得出的重力加速度
float accel_scale;          // 根据标定结果校准加速度计标度因数
// 需定期校准后手动修改
#define GxOFFSET  0.00000127707f
#define GyOFFSET -0.00000808811f
#define GzOFFSET -0.00002852123f
#define gNORM 9.744925f

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void bmi088_gyro_rotate_to_frd(float* data)
{
    /* do nothing */
    (void)data;
}

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void bmi088_acc_rotate_to_frd(float* data)
{
    /* do nothing */
    (void)data;
}

static rt_err_t __write_checked_reg(rt_device_t spi_device, rt_uint8_t reg, rt_uint8_t val)
{
    rt_uint8_t r_val;

    spi_write_reg8(spi_device, reg, val);
    /* In case of read operations of the accelerometer part, the requested data is not sent
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual
    reqested register content is transmitted. */
    spi_read_reg8(spi_device, reg, &r_val);
    spi_read_reg8(spi_device, reg, &r_val);

    return (r_val == val) ? RT_EOK : RT_ERROR;
}

static rt_err_t __modify_reg(rt_device_t spi_device, rt_uint8_t reg, reg_val_t reg_val)
{
    uint8_t value;

    /* In case of read operations of the accelerometer part, the requested data is not sent
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual
    reqested register content is transmitted. */
    spi_read_reg8(spi_device, reg, &value);
    spi_read_reg8(spi_device, reg, &value);

    value &= ~reg_val.clearbits;
    value |= reg_val.setbits;

    __write_checked_reg(spi_device, reg, value);

    return RT_EOK;
}

/* ---------------------------------- GYRO ---------------------------------- */

static rt_err_t gyro_set_sample_rate(uint32_t frequency_hz)
{
    reg_val_t reg_val;

    if (frequency_hz <= 150) {
        reg_val = BMI088_GYRO_RATE_100;
    } else if (frequency_hz <= 300) {
        reg_val = BMI088_GYRO_RATE_200;
    } else if (frequency_hz <= 700) {
        reg_val = BMI088_GYRO_RATE_400;
    } else if (frequency_hz <= 1500) {
        reg_val = BMI088_GYRO_RATE_1000;
    } else if (frequency_hz <= 2000) {
        reg_val = BMI088_GYRO_RATE_2000;
    } else {
        return RT_EINVAL;
    }

    __modify_reg(gyro_spi_dev, BMI088_BW_ADDR | BMI088_GYRO_BANDWIDTH_MUST_Set, reg_val);

    return RT_EOK;
}

static rt_err_t gyro_set_dlpf_filter(uint16_t frequency_hz)
{
    /* lpf bw is set by BMI088_BW_ADDR */
    (void)frequency_hz;

    return RT_EOK;
}

static rt_err_t gyro_set_range(unsigned max_dps)
{
    reg_val_t reg_val;
    float lsb_per_dps;

    if (max_dps == 0) {
        max_dps = 2000;
    }
    if (max_dps <= 187) {
        reg_val = BMI088_GYRO_RANGE_125_DPS;
        lsb_per_dps = 262.4;
    } else if (max_dps <= 375) {
        reg_val = BMI088_GYRO_RANGE_250_DPS;
        lsb_per_dps = 131.2;
    } else if (max_dps <= 750) {
        reg_val = BMI088_GYRO_RANGE_500_DPS;
        lsb_per_dps = 65.6;
    } else if (max_dps <= 1500) {
        reg_val = BMI088_GYRO_RANGE_1000_DPS;
        lsb_per_dps = 32.8;
    } else if (max_dps <= 2000) {
        reg_val = BMI088_GYRO_RANGE_2000_DPS;
        lsb_per_dps = 16.4;
    } else {
        return RT_EINVAL;
    }

    __modify_reg(gyro_spi_dev, BMI088_RANGE_ADDR, reg_val);

    gyro_range_scale = (M_PI_F / (180.0f * lsb_per_dps));

    return RT_EOK;
}

static rt_err_t gyro_read_raw(int16_t gyr[3])
{
    spi_read_multi_reg8(gyro_spi_dev, BMI088_RATE_X_LSB_ADDR, (uint8_t*)gyr, 6);

    return RT_EOK;
}

static rt_err_t gyro_read_rad(float gyr[3])
{
    int16_t gyr_raw[3];

    gyro_read_raw(gyr_raw);

    gyr[0] = gyro_range_scale * gyr_raw[0] - gyro_offset[0];
    gyr[1] = gyro_range_scale * gyr_raw[1] - gyro_offset[1];
    gyr[2] = gyro_range_scale * gyr_raw[2] - gyro_offset[2];

    return RT_EOK;
}

static rt_err_t gyroscope_init(void)
{
    uint8_t gyro_id;

    /* init spi bus */
    rt_device_open(gyro_spi_dev, RT_DEVICE_OFLAG_RDWR);

    spi_read_reg8(gyro_spi_dev, BMI088_CHIP_ID_ADDR, &gyro_id);
    if (gyro_id != BMI088_GRRO_CHIP_ID) {
        LOG_W("Warning: not found BMI088 gyro id: %02x", gyro_id);
        return RT_ERROR;
    }

    /* soft reset */
    spi_write_reg8(gyro_spi_dev, BMI088_BGW_SOFT_RST_ADDR, 0xB6);
    rt_hw_us_delay(35000); // > 30ms delay

    gyro_set_range(2000);       /* 2000dps */
    gyro_set_sample_rate(2000); /* OSR 2000KHz, Filter BW: 230Hz */

    /* enable gyroscope */
    __modify_reg(gyro_spi_dev, BMI088_MODE_LPM1_ADDR, REG_VAL(0, BIT(7) | BIT(5))); /* {0; 0}  NORMAL mode */
    rt_hw_us_delay(1000);

    return RT_EOK;
}

/**
 * @brief 设置BMI088的陀螺仪配置
 *
 * @param cfg 配置参数
 * @return rt_err_t
 */
static rt_err_t bim088_gyro_config(struct gyro_configure cfg)
{
    gyro_set_sample_rate(cfg.sample_rate_hz);
    gyro_set_dlpf_filter(cfg.dlpf_freq_hz);
    gyro_set_range(cfg.gyro_range_dps);

    return RT_EOK;
}

/**
 * @brief 读取BMI088的加速度计数据
 *
 * @param data 读取到的数据
 * @return rt_err_t
 */
static rt_err_t bim088_gyro_read(float data[3])
{
    if (gyro_read_rad(data) != RT_EOK) {
        return -RT_ERROR;
    }
    // change to NED coordinate
    bmi088_gyro_rotate_to_frd(data);

    return RT_EOK;
}
/* ---------------------------------- GYRO ---------------------------------- */

/* ---------------------------------- ACCEL --------------------------------- */
static rt_err_t accel_set_sample_rate(uint32_t frequency_hz)
{
    reg_val_t reg_val;

    if (frequency_hz <= (125 / 10)) {
        reg_val = BMI088_ACCEL_RATE_12_5;
        sample_rate = 12.5;
    } else if (frequency_hz <= 25) {
        reg_val = BMI088_ACCEL_RATE_25;
        sample_rate = 25;
    } else if (frequency_hz <= 50) {
        reg_val = BMI088_ACCEL_RATE_50;
        sample_rate = 50;
    } else if (frequency_hz <= 100) {
        reg_val = BMI088_ACCEL_RATE_100;
        sample_rate = 100;
    } else if (frequency_hz <= 200) {
        reg_val = BMI088_ACCEL_RATE_200;
        sample_rate = 200;
    } else if (frequency_hz <= 400) {
        reg_val = BMI088_ACCEL_RATE_400;
        sample_rate = 400;
    } else if (frequency_hz <= 800) {
        reg_val = BMI088_ACCEL_RATE_800;
        sample_rate = 800;
    } else if (frequency_hz <= 1600) {
        reg_val = BMI088_ACCEL_RATE_1600;
        sample_rate = 1600;
    } else {
        return -RT_EINVAL;
    }

    __modify_reg(accel_spi_dev, BMI088_ACC_CONF, reg_val);

    return RT_EOK;
}

static rt_err_t accel_set_bwp_odr(uint16_t dlpf_freq_hz)
{
    reg_val_t reg_val;

    if (sample_rate <= 12.5) {
        if (dlpf_freq_hz <= 1) {
            // 1Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 3) {
            // 2Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 5Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 25) {
        if (dlpf_freq_hz <= 4) {
            // 3Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 7) {
            // 5Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 10Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 50) {
        if (dlpf_freq_hz <= 7) {
            // 5Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 14) {
            // 9Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 20Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 100) {
        if (dlpf_freq_hz <= 14) {
            // 10Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 29) {
            // 19Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 40Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 200) {
        if (dlpf_freq_hz <= 29) {
            // 20Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 59) {
            // 38Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 80Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 400) {
        if (dlpf_freq_hz <= 52) {
            // 40Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 110) {
            // 75Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 145Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 800) {
        if (dlpf_freq_hz <= 110) {
            // 80Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 175) {
            // 140Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 230Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else if (sample_rate <= 1600) {
        if (dlpf_freq_hz <= 199) {
            // 145Hz
            reg_val = BMI088_ACCEL_OSR_4;
        }
        if (dlpf_freq_hz <= 257) {
            // 234Hz
            reg_val = BMI088_ACCEL_OSR_2;
        } else {
            // 280Hz
            reg_val = BMI088_ACCEL_OSR_0;
        }
    } else {
        return -RT_EINVAL;
    }

    __modify_reg(accel_spi_dev, BMI088_ACC_CONF, reg_val);
    return RT_EOK;
}

static rt_err_t accel_set_range(uint32_t max_g)
{
    uint8_t reg_val;

    if (max_g == 0) {
        max_g = 24;
    }
    if (max_g <= 3) {
        reg_val = BMI088_ACCEL_RANGE_3_G;
        accel_range_scale = (3 * BMI088_ONE_G / 32768);
    } else if (max_g <= 6) {
        reg_val = BMI088_ACCEL_RANGE_6_G;
        accel_range_scale = (6 * BMI088_ONE_G / 32768);
    } else if (max_g <= 12) {
        reg_val = BMI088_ACCEL_RANGE_12_G;
        accel_range_scale = (12 * BMI088_ONE_G / 32768);
    } else if (max_g <= 24) {
        reg_val = BMI088_ACCEL_RANGE_24_G;
        accel_range_scale = (24 * BMI088_ONE_G / 32768);
    } else {
        return RT_EINVAL;
    }

    spi_write_reg8(accel_spi_dev, BMI088_ACC_RANGE, reg_val);
    return RT_EOK;
}

static rt_err_t accelerometer_init(void)
{
    uint8_t accel_id;

    /* init spi bus */
    rt_device_open(accel_spi_dev, RT_DEVICE_OFLAG_RDWR);

    /* dummy read to let accel enter SPI mode */
    spi_read_reg8(accel_spi_dev, BMI088_ACC_BGW_CHIPID, &accel_id);
    rt_hw_us_delay(1000);
    spi_read_reg8(accel_spi_dev, BMI088_ACC_BGW_CHIPID, &accel_id);

    /* read accel id */
    spi_read_reg8(accel_spi_dev, BMI088_ACC_BGW_CHIPID, &accel_id);
    if (accel_id != BMI088_ACC_BGW_CHIPID_VALUE) {
        LOG_W("Warning: not found BMI088 accel id: %02x", accel_id);
        return RT_ERROR;
    }

    /* soft reset */
    spi_write_reg8(accel_spi_dev, BMI088_ACC_SOFTRESET, 0xB6);
    rt_hw_us_delay(2000);
    /* dummy read to let accel enter SPI mode */
    spi_read_reg8(accel_spi_dev, BMI088_ACC_BGW_CHIPID, &accel_id);
    /* enter normal mode */
    spi_write_reg8(accel_spi_dev, BMI088_ACC_PWR_CTRL, 0x04);
    rt_hw_us_delay(55000);

    /* set default range and bandwidth */
    accel_set_range(6);          /* 6g */
    accel_set_sample_rate(800);  /* 800Hz sample rate */
    accel_set_bwp_odr(280);      /* Normal BW */

    /* enter active mode */
    spi_write_reg8(accel_spi_dev, BMI088_ACC_PWR_CONF, 0x00);
    rt_hw_us_delay(1000);

    return RT_EOK;
}

/**
 * @brief 读取BMI088的温度
 *
 * @return float 温度（摄氏度）
 */
static float bmi088_temp_read(void)
{
    uint8_t buffer[2];
    static int16_t raw_temp;
    static float temp;

    spi_read_multi_reg8(accel_spi_dev, BMI088_INT_TEMP_MSB, buffer, 2);
    raw_temp = (int16_t)((buffer[1] << 3) | (buffer[0] >> 5));
    if (raw_temp > 1023)
    {
        raw_temp -= 2048;
    }
    temp = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return temp;
}

static rt_err_t accel_read_raw(int16_t acc[3])
{
    uint8_t buffer[7];

    /* In case of read operations of the accelerometer part, the requested data is not sent
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual
    reqested register content is transmitted. */
    spi_read_multi_reg8(accel_spi_dev, BMI088_ACCD_X_LSB, buffer, 7);

    acc[0] = buffer[2] << 8 | buffer[1];
    acc[1] = buffer[4] << 8 | buffer[3];
    acc[2] = buffer[6] << 8 | buffer[5];

    return RT_EOK;
}

static rt_err_t accel_read_m_s2(float acc[3])
{
    int16_t acc_raw[3];

    accel_read_raw(acc_raw);

    acc[0] = accel_range_scale * acc_raw[0] * accel_scale;
    acc[1] = accel_range_scale * acc_raw[1] * accel_scale;
    acc[2] = accel_range_scale * acc_raw[2] * accel_scale;

    return RT_EOK;
}

/**
 * @brief 设置BMI088的加速度计配置
 *
 * @param cfg 配置参数
 * @return rt_err_t
 */
static  rt_err_t bim088_accel_config(struct accel_configure cfg)
{
    accel_set_sample_rate(cfg.sample_rate_hz);
    accel_set_bwp_odr(cfg.dlpf_freq_hz);
    accel_set_range(cfg.acc_range_g);

    return RT_EOK;
}

/**
 * @brief 读取BMI088的陀螺仪数据
 *
 * @param data 读取到的数据
 * @return rt_err_t
 */
static rt_err_t bim088_accel_read(float data[3])
{
    if (accel_read_m_s2(data) != RT_EOK) {
        return -RT_ERROR;;
    }
    // change to NED coordinate
    bmi088_acc_rotate_to_frd(data);

    return RT_EOK;
}
/* ---------------------------------- ACCEL --------------------------------- */

/* -------------------------------- CALIBRATE ------------------------------- */
/**
 * @brief bmi088 校准函数
 *
 * @note 定期或更换开发板时进行一次校准即可,校准成功后手动修改 GxOFFSET 等宏;
 *       通过在 menuconfig 中使能 BSP_BMI088_CALI 进行校准;
 *       在串口终端可以查看校准进度,如多次校准失败,适当调大误差范围;
 *
 * @return 校准成功将校准值写入 gyro_offset
 */
static void bmi088_calibrate(void){
    static float start_time;
    static uint16_t cali_times = 5000;   // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    float accel[3], gyro[3];
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    static float gyroDiff[3], gNormDiff;
    int16_t acc_raw[3];

    start_time = dwt_get_time_s();
    do
    {
        if (dwt_get_time_s() - start_time > 20)
        {
            // 校准超时
/*            gyro_offset[0] = GxOFFSET;
            gyro_offset[1] = GyOFFSET;
            gyro_offset[2] = GzOFFSET;
            bmi088_g_norm = gNORM;*/
            break;
        }

        dwt_delay_s(0.005);
        // 开始时先置零，避免对数据读取造成影响
        bmi088_g_norm = 0;
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++)
        {
            accel_read_raw(acc_raw);
            accel[0] = accel_range_scale * acc_raw[0];
            accel[1] = accel_range_scale * acc_raw[1];
            accel[2] = accel_range_scale * acc_raw[2];
            gNormTemp = sqrtf(accel[0] * accel[0] +
                              accel[1] * accel[1] +
                              accel[2] * accel[2]);
            bmi088_g_norm += gNormTemp;

            gyro_read_rad(gyro);
            for(uint8_t j = 0; j < 3; j++){
                gyro_offset[j] += gyro[j];
            }

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = gyro[j];
                    gyroMin[j] = gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (gyro[j] > gyroMax[j])
                        gyroMax[j] = gyro[j];
                    if (gyro[j] < gyroMin[j])
                        gyroMin[j] = gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.6f ||
                gyroDiff[0] > 1.0f ||
                gyroDiff[1] > 1.0f ||
                gyroDiff[2] > 1.0f)
                break;
            LOG_I("gyroDiff: %f",gNormDiff);
            for(uint8_t j = 0; j < 3; j++){
                LOG_D("gyroDiff%d: %f",j ,gyroDiff[j]);
            }
            dwt_delay_s(0.0005);
        }
        // 取平均值得到标定结果
        bmi088_g_norm /= (float)cali_times;
        LOG_W("bmi088_g_norm: %f",bmi088_g_norm);
        for (uint8_t i = 0; i < 3; i++)
        {
            gyro_offset[i] /= (float)cali_times;
            LOG_W("gyro_offset: %f",gyro_offset[i]);
        }

        cali_count++;
    } while (gNormDiff > 0.3f ||
             fabsf(bmi088_g_norm - 9.8f) > 0.5f ||
             gyroDiff[0] > 1.0f ||
             gyroDiff[1] > 1.0f ||
             gyroDiff[2] > 1.0f ||
             fabsf(gyro_offset[0]) > 0.01f ||
             fabsf(gyro_offset[1]) > 0.01f ||
             fabsf(gyro_offset[2]) > 0.01f);
    // 根据标定结果校准加速度计标度因数
    accel_scale = 9.81f / bmi088_g_norm;
}
/* -------------------------------- CALIBRATE ------------------------------- */

/**
 * @brief 初始化BMI088
 *
 * @return rt_err_t
 */
static rt_err_t bmi088_init(void)
{
    /* Initialize accelerometer */
    rt_hw_spi_device_attach(SPI_ACC, "bmi088_a", SPI_ACC_CS);
    accel_spi_dev = rt_device_find("bmi088_a");
    RT_ASSERT(accel_spi_dev != NULL);
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 7000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)accel_spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        rt_spi_configure(spi_device_t, &cfg);
    }
    /* accelerometer low-level init */
    accelerometer_init();

    /* Initialize gyroscope */
    rt_hw_spi_device_attach(SPI_GYRO, "bmi088_g", SPI_GYRO_CS);
    gyro_spi_dev = rt_device_find("bmi088_g");
    RT_ASSERT(gyro_spi_dev != NULL);
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 7000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)gyro_spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        rt_spi_configure(spi_device_t, &cfg);
    }
    /* gyroscope low-level init */
    gyroscope_init();
    gyro_offset[0] = GxOFFSET;
    gyro_offset[1] = GyOFFSET;
    gyro_offset[2] = GzOFFSET;
    bmi088_g_norm = gNORM;
    accel_scale = 9.81f / bmi088_g_norm;
    /* calibrate */
#ifdef BSP_BMI088_CALI
    bmi088_calibrate();
#endif /* BSP_BMI088_CALI */

    return RT_EOK;
}

struct imu_ops imu_ops = {
    .imu_init = bmi088_init,
    .gyro_read = bim088_gyro_read,
    .gyro_config = bim088_gyro_config,
    .accel_read = bim088_accel_read,
    .accel_config = bim088_accel_config,
    .temp_read = bmi088_temp_read,
};
