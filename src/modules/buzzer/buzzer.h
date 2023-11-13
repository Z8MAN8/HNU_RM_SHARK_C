/*
 * Change Logs:
 * Date           Author       Notes
 * 2023-10-18     MaSongdong   first version
 */
#ifndef RTTHREAD_BUZZER_H
#define RTTHREAD_BUZZER_H

#include <rtthread.h>
#include <rtdevice.h>

typedef struct buzzer_obj{
    struct rt_device_pwm* pwm_dev;
    uint8_t ring_flag;  // 为1时蜂鸣器正处于响的状态
    uint8_t pwm_channel;
    void (*buzzer_on)(struct buzzer_obj *obj, int8_t times, rt_int32_t delay_ms, rt_uint32_t voice, rt_uint32_t period);
    void (*buzzer_off)(struct buzzer_obj *obj);
}buzzer_obj_t;

buzzer_obj_t *buzzer_register(const char *pwm_name, int8_t pwm_channel);

#endif // RTTHREAD_BUZZER_H
