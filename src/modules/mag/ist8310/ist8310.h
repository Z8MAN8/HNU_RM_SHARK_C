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
#ifndef IST8310_H__
#define IST8310_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "mag.h"

extern struct mag_ops mag;

/**
 * @brief 磁力计校准函数，如需要校准，用户自定义即可
 *
 * @param data[3] 读取到的磁力计数据
 */
__attribute__((weak)) void ist8310_user_calibrate(float data[3]);

/**
 * @brief Re-implement this function to define customized rotation
 *
 * @param data 读取到的磁力计数据
 */
__attribute__((weak)) void ist8310_rotate_to_frd(float* data);

#endif /* IST8310_H__ */
