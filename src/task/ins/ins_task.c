/*
* Change Logs:
* Date            Author          Notes
* 2023-09-15      ChuShicheng     first version
*/

#include "ins_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "ins.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define TEMP_PWM_DEV_NAME        "pwm10"  /* PWM设备名称 */
#define TEMP_PWM_DEV_CHANNEL      1       /* PWM通道 */
#define IMU_TARGET_TEMP           40      /* imu期望恒温温度 */

static struct rt_device_pwm *temp_pwm_dev;      /* 温度PWM设备句柄 */
static rt_uint32_t period = 250000;
/*static*/ rt_uint32_t pulse = 0;

/*static*/ float gyro[3],acc[3],temp;
 float gyro_total[3],acc_total[3];

/*static*/ pid_obj_t *imu_temp_pid;
static pid_config_t imu_temp_config = {
        .Kp = 50000, // 4.5
        .Ki = 8000,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 50000,
        .Improve = PID_Integral_Limit,
        .MaxOut = 250000,
};

static rt_err_t temp_pwm_init(rt_uint32_t period, rt_uint32_t pulse);

void ins_thread_entry(void *argument)
{
    static float example_dt;
    static float example_start;

    temp_pwm_init(period, pulse);
    /* 注册 PID 实例 */
    imu_temp_pid = pid_register(&imu_temp_config);
    imu_ops.imu_init();
    LOG_I("Example Task Start");

    for (;;)
    {
        example_start = dwt_get_time_ms();
        imu_ops.gyro_read(gyro);
        imu_ops.accel_read(acc);
        temp = imu_ops.temp_read();
        pulse = pid_calculate(imu_temp_pid, temp, IMU_TARGET_TEMP);
        rt_pwm_set_pulse(temp_pwm_dev, TEMP_PWM_DEV_CHANNEL, pulse);
         for (uint8_t j = 0; j < 3; j++){
            gyro_total[j] += gyro[j];
            acc_total[j] += acc[j];
        }
        /* 用于调试监测线程调度使用 */
        example_dt = dwt_get_time_ms() - example_start;
        if (example_dt > 1)
            LOG_E("Example Task is being DELAY! dt = [%f]", &example_dt);

        rt_thread_delay(1);
    }
}

static rt_err_t temp_pwm_init(rt_uint32_t period, rt_uint32_t pulse)
{
    temp_pwm_dev = (struct rt_device_pwm *)rt_device_find(TEMP_PWM_DEV_NAME);
    if (temp_pwm_dev == RT_NULL)
    {
        LOG_E("Can't find %s device!", TEMP_PWM_DEV_NAME);
        return -RT_ERROR;
    }
    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(temp_pwm_dev, TEMP_PWM_DEV_CHANNEL, period, pulse);
    /* 使能设备 */
    rt_pwm_enable(temp_pwm_dev, TEMP_PWM_DEV_CHANNEL);
}
