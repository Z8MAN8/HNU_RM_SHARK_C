#include "motor_task.h"
#include "dji_motor.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static float motor_dt;

void motor_thread_entry(void *argument)
{
    static float motor_start;
    LOG_I("Motor Task Start");

    motor_start = dwt_get_time_ms();

    for (;;)
    {
        motor_start = dwt_get_time_ms();
        // static uint8_t cnt = 0; 设定不同电机的任务频率
        // if(cnt%5==0) //200hz
        // if(cnt%10==0) //100hz
#ifdef BSP_USING_DJI_MOTOR
        dji_motor_control();
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_LK_MOTOR
        lk_motor_control();
#endif /* BSP_USING_LK_MOTOR */
#ifdef BSP_USING_HT_MOTOR
        ht_motor_control();
#endif /* BSP_USING_HT_MOTOR */

        /* 用于调试监测线程调度使用 */
        motor_dt = dwt_get_time_ms() - motor_start;
        if (motor_dt > 3){
           rt_kprintf("Motor Task is being DELAY! dt = [%f]\n", &motor_dt);
        }
        rt_thread_delay(1);
    }

}
