#include "motor_task.h"
#include "dji_motor.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

void motor_thread_entry(void *argument)
{
    static float motor_dt;
    static float motor_start;
    LOG_I("Motor Task Start");

    motor_start = dwt_get_time_ms();

    for (;;)
    {
        motor_start = dwt_get_time_ms();
        // static uint8_t cnt = 0; 设定不同电机的任务频率
        // if(cnt%5==0) //200hz
        // if(cnt%10==0) //100hz
        dji_motor_control();

        /* 如果有对应的电机则取消注释,可以加入条件编译或者register对应的idx判断是否注册了电机 */
        // LKMotorControl();

        ht_motor_control();

        /* 用于调试监测线程调度使用 */
        motor_dt = dwt_get_time_ms() - motor_start;
        if (motor_dt > 1){
//            rt_kprintf("Motor Task is being DELAY! dt = [%f]\n", &motor_dt);
        }
        rt_thread_delay(1);
    }

}
