//
// Created by turboDog on 2021/11/21.
//

#ifndef RTTHREAD_RAMP_H
#define RTTHREAD_RAMP_H

#include <rtthread.h>

#define RAMP_NUM_MAX 30      // 最大RAMP实例数

typedef struct ramp_config_t ramp_config_t;

struct ramp_config_t
{
    int32_t count;//计数器
    int32_t scale;//控制数据变化斜率
    float   out;
    void  (*reset)(struct ramp_config_t* ramp, int32_t count,int32_t scale);
    float (*calc)(struct ramp_config_t* ramp);
};

/**
 * @brief 初始化ramp实例,并返回ramp实例指针
 * @param config PID初始化设置
 */
ramp_config_t *ramp_register(int32_t count,int32_t scale);

#endif //RTTHREAD_RAMP_H
