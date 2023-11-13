/*
 * Change Logs:
 * Date           Author       Notes
 * 2023-10-18     MaSongdong   first version
 */
#include "buzzer.h"
#include "rm_config.h"

#define DBG_TAG   "modules.buzzer"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static buzzer_obj_t *buzzer_obj = NULL;

/**
 * @brief 蜂鸣器启动
 *
 * @param times 鸣叫次数
 * @param delay_ms 鸣叫时间(ms)
 * @param voice 提示音量(0~100)
 * @param period 频率(建议70000~1000000)
 */
static void buzzer_on(buzzer_obj_t *obj, int8_t times, rt_int32_t delay_ms, rt_uint32_t voice, rt_uint32_t period)
{
    obj->ring_flag =1;
    struct rt_device_pwm* pwm_dev = obj->pwm_dev;

    rt_pwm_enable(pwm_dev,obj->pwm_channel);
    for(int i=1; i <= times; i++)
    {
        rt_pwm_set(pwm_dev, obj->pwm_channel, period, voice*period/100*0.9);
        rt_thread_mdelay(delay_ms);
        rt_pwm_set(pwm_dev, obj->pwm_channel, period, 0);
        rt_thread_mdelay(delay_ms);
    }
}

static void buzzer_off(buzzer_obj_t *obj)
{
    obj->ring_flag =0;
    rt_pwm_disable(obj->pwm_dev, obj->pwm_channel);
}


buzzer_obj_t *buzzer_register(const char *pwm_name, int8_t pwm_channel){
    buzzer_obj_t *object = (buzzer_obj_t *)rt_malloc(sizeof(buzzer_obj_t));
    rt_memset(object, 0, sizeof(buzzer_obj_t));

    struct rt_device_pwm* pwm_dev = (struct rt_device_pwm*)rt_device_find(pwm_name);
    if(pwm_dev == RT_NULL) {
        LOG_W("%s not found", pwm_name);
    }
    object->pwm_channel = pwm_channel;
    object->pwm_dev = pwm_dev;
    object->buzzer_on = buzzer_on;
    object->buzzer_off = buzzer_off;
    buzzer_obj = object;

    return object;
}
