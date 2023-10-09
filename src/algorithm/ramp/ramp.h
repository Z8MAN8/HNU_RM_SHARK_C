//
// Created by 欧阳小枫 on 2023/10/7.
//

#ifndef RTTHREAD_RAMP_H
#define RTTHREAD_RAMP_H

#include "stm32f4xx_hal.h"

typedef struct ramp_t
{
    int32_t count;
    int32_t scale;
    float   out;
    void  (*init)(struct ramp_t *ramp, int32_t scale);
    float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT \
{ \
  .count = 0, \
  .scale = 0, \
  .out = 0, \
  .init = &ramp_init, \
  .calc = &ramp_calc, \
} \
/**
  * @brief     斜坡控制结构体初始化
  * @param[in] ramp: 斜坡数据结构体指针
  * @param[in] scale: 控制数据变化斜率
  */
void  ramp_init(ramp_t *ramp, int32_t scale);
/**
  * @brief     斜坡控制计算函数
  * @param[in] ramp: 斜坡数据结构体指针
  * @retval    斜坡控制计算输出
  */
float ramp_calc(ramp_t *ramp);

/* yaw 轴云台控制斜坡 */
extern ramp_t yaw_ramp;
/* pitch 轴云台控制斜坡 */
extern ramp_t pit_ramp;

#endif //RTTHREAD_RAMP_H
