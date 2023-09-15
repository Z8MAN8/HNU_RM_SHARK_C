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
#ifndef BMI088_H__
#define BMI088_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct imu_ops imu_ops;
extern float bmi088_g_norm;   // 通过校准得出的重力加速度,数据融合时会用到

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void bmi088_acc_rotate_to_frd(float* data);
/* Re-implement this function to define customized rotation */
__attribute__((weak)) void bmi088_gyro_rotate_to_frd(float* data);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H__ */
