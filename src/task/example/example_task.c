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
}chassis_controller[2];

static ht_motor_object_t *ht_chassis_motor[2];
static lk_motor_object_t *lk_chassis_motor[2];

static int16_t lk_control1(lk_motor_measure_t measure){
    static int16_t set;

    set = pid_calculate(chassis_controller[0].speed_pid, measure.speed_rads, measure.target);

    return set;
}

static int16_t lk_control2(lk_motor_measure_t measure){
    static int16_t set;

    set = pid_calculate(chassis_controller[0].speed_pid, measure.speed_rads, measure.target);

    return set;
}


static ht_motor_para_t chassis_control(ht_motor_measure_t measure){
    static ht_motor_para_t set;

    {
        set.p = 0;
        set.kp = 0;
        set.v = -1;
        set.kd = 1;
        set.t = 0;
    }
    return set;
}

static ht_motor_para_t chassis_control2(ht_motor_measure_t measure){
    static ht_motor_para_t set;

    {
        set.p = 0;
        set.kp = 0;
        set.v = 4;
        set.kd = 1;
        set.t = 0;
    }
    return set;
}

static void example_init()
{
    pid_config_t lk_speed_config = {
            .Kp = 10, // 4.5
            .Ki = 0,  // 0
            .Kd = 0,  // 0
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 12000,
    };
    chassis_controller[0].speed_pid = pid_register(&lk_speed_config);
    chassis_controller[1].speed_pid = pid_register(&lk_speed_config);

/*    motor_config_t chassis_motor_config = {
            .motor_type = HT04,
            .can_name = CAN_CHASSIS,
            .tx_id = 0x01,
            .rx_id = 0x10,
            .controller = &chassis_controller,
    };
    motor_config_t chassis_motor2_config = {
            .motor_type = HT04,
            .can_name = CAN_CHASSIS,
            .tx_id = 0x02,
            .rx_id = 0x10,
            .controller = &chassis_controller,
    };
    ht_chassis_motor[0] = ht_motor_register(&chassis_motor_config, chassis_control);
    ht_chassis_motor[1] = ht_motor_register(&chassis_motor2_config, chassis_control2);*/

    motor_config_t lk_motor1_config = {
            .motor_type = MF9025,
            .can_name = CAN_CHASSIS,
            .tx_id = 0x280,
            .rx_id = 0x141,
            .controller = &chassis_controller[0],
    };
    motor_config_t lk_motor2_config = {
            .motor_type = MF9025,
            .can_name = CAN_CHASSIS,
            .tx_id = 0x280,
            .rx_id = 0x142,
            .controller = &chassis_controller[1],
    };
    lk_chassis_motor[0] = lk_motor_register(&lk_motor1_config, lk_control1);
//    lk_chassis_motor[1] = lk_motor_register(&lk_motor2_config, lk_control2);
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
