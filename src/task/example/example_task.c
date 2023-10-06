/*
* Change Logs:
* Date            Author          Notes
* 2023-09-05      ChuShicheng     first version
*/

#include "example_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct chassis_controller_t{
    pid_obj_t *speed_pid;
}chassis_controller;

static struct gimbal_controller_t{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid;
}gimbal_controlelr;

static dji_motor_object_t *chassis_motor;
static dji_motor_object_t *gimbal_motor;

static rt_int16_t chassis_control(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller.speed_pid, measure.speed_rpm, 1000);
    return set;
}

static rt_int16_t gimbal_control(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr.speed_pid, measure.speed_rpm, 0);
    return set;
}

static void example_init()
{
    pid_config_t chassis_speed_config = {
            .Kp = 10, // 4.5
            .Ki = 0,  // 0
            .Kd = 0,  // 0
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 12000,
    };
    pid_config_t gimbal_speed_config = {
            .Kp = 50,  // 50
            .Ki = 200, // 200
            .Kd = 0,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3000,
            .MaxOut = 20000,
    };
    chassis_controller.speed_pid = pid_register(&chassis_speed_config);
    gimbal_controlelr.speed_pid = pid_register(&gimbal_speed_config);

    motor_config_t chassis_motor_config = {
            .motor_type = M3508,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x201,
            .controller = &chassis_controller,
    };
    motor_config_t gimbal_motor_config = {
            .motor_type = GM6020,
            .can_name = CAN_GIMBAL,
            .rx_id = 0x206,
            .controller = &gimbal_controlelr,
    };
    chassis_motor = dji_motor_register(&chassis_motor_config, chassis_control);
    gimbal_motor = dji_motor_register(&gimbal_motor_config, gimbal_control);
}

void example_thread_entry(void *argument)
{
    static float example_dt;
    static float example_start;
    example_init();
    LOG_I("Example Task Start");
    for (;;)
    {
        example_start = dwt_get_time_ms();

        /* 用于调试监测线程调度使用 */
        example_dt = dwt_get_time_ms() - example_start;
        if (example_dt > 1)
            LOG_E("Example Task is being DELAY! dt = [%f]", &example_dt);

        rt_thread_delay(1);
    }
}
