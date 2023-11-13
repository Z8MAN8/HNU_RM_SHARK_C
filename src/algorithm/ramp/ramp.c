//
// Created by turboDog on 2021/11/21.
//

#include "ramp.h"

static uint8_t idx = 0; // register idx,是该文件的全局ramp索引,在注册时使用
/* ramp斜坡算法的实例,此处仅保存指针,内存的分配将通过实例初始化时通过malloc()进行 */
static ramp_obj_t *ramp_obj[RAMP_NUM_MAX] = {NULL};

/**
  * @brief     斜坡控制实例重置
  * @param[in] ramp: 斜坡实例指针
  * @param[in] scale: 控制数据变化斜率
  */
void ramp_reset(ramp_obj_t *ramp, int32_t count,int32_t scale)
{
    ramp->count = count;
    ramp->scale = scale;
}

/**
  * @brief     斜坡控制计算函数
  * @param[in] ramp: 斜坡实例指针
  * @retval    斜坡控制计算输出
  */
float ramp_calc(ramp_obj_t *ramp)
{
    if (ramp->scale <= 0)
        return 0;

    if (ramp->count++ >= ramp->scale)
        ramp->count = ramp->scale;

    ramp->out = ramp->count / ((float)ramp->scale);
    return ramp->out;
}

/**
 * @brief 初始化ramp实例,并返回ramp实例指针
 * @param config PID初始化设置
 */
ramp_obj_t *ramp_register(int32_t count,int32_t scale)
{
    ramp_obj_t *object = (ramp_obj_t *)rt_malloc(sizeof(ramp_obj_t));
    rt_memset(object, 0, sizeof(ramp_obj_t));

    object->count = count;
    object->scale = scale;
    object->reset = &ramp_reset;
    object->calc = &ramp_calc;

    ramp_obj[idx++] = object;
    return object;
}
