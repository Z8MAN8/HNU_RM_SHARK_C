 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-28      ChuShicheng     first version
 */
#include <rtthread.h>
#include "rm_task.h"

#define DBG_TAG   "robot"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_EXAMPLE_TASK
rt_thread_t example_thread_handle;
#endif /* BSP_USING_EXAMPLE_TASK */
#ifdef BSP_USING_INS_TASK
rt_thread_t ins_thread_handle;
#endif /* BSP_USING_INS_TASK */
#ifdef BSP_USING_MOTOR_TASK
 rt_thread_t motor_thread_handle;
#endif /* BSP_USING_MOTOR_TASK */
#ifdef BSP_USING_CMD_TASK
 rt_thread_t cmd_thread_handle;
#endif /* BSP_USING_CMD_TASK */
#ifdef BSP_USING_CHASSIS_TASK
 rt_thread_t chassis_thread_handle;
#endif /* BSP_USING_GIMBAL_TASK */
#ifdef BSP_USING_GIMBAL_TASK
 rt_thread_t gimbal_thread_handle;
#endif /* BSP_USING_GIMBAL_TASK */

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
int robot_task_init(void)
{
#ifdef BSP_USING_EXAMPLE_TASK
    /* 创建线程，名称是 example，入口是 example_thread_handle */
    example_thread_handle = rt_thread_create("example",
                                             example_thread_entry, RT_NULL,
                                             1024,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (example_thread_handle != RT_NULL)
        rt_thread_startup(example_thread_handle);
#endif /* BSP_USING_EXAMPLE_TASK */

#ifdef BSP_USING_INS_TASK
    /* 创建线程，名称是 ins ，入口是 ins_thread_entry */
    ins_thread_handle = rt_thread_create("ins",
                                         ins_thread_entry, RT_NULL,
                                         2048,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (ins_thread_handle != RT_NULL)
       rt_thread_startup(ins_thread_handle);
#endif /* BSP_USING_INS_TASK */

#ifdef BSP_USING_MOTOR_TASK
    /* 创建线程，名称是 motor ，入口是 motor_thread_entry */
    motor_thread_handle = rt_thread_create("motor",
                                            motor_thread_entry, RT_NULL,
                                            2048,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (motor_thread_handle != RT_NULL)
        rt_thread_startup(motor_thread_handle);
#endif /* BSP_USING_MOTOR_TASK */

#ifdef BSP_USING_CMD_TASK
    /* 创建线程，名称是 cmd ，入口是 cmd_thread_entry */
    cmd_thread_handle = rt_thread_create("cmd",
                                          cmd_thread_entry, RT_NULL,
                                          1024,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (cmd_thread_handle != RT_NULL)
        rt_thread_startup(cmd_thread_handle);
#endif /* BSP_USING_CMD_TASK */

#ifdef BSP_USING_CHASSIS_TASK
    /* 创建线程，名称是 chassis ，入口是 chassis_thread_entry */
    chassis_thread_handle = rt_thread_create("chassis",
                                              chassis_thread_entry, RT_NULL,
                                              1024,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (chassis_thread_handle != RT_NULL)
        rt_thread_startup(chassis_thread_handle);
#endif /* BSP_USING_CHASSIS_TASK */

#ifdef BSP_USING_GIMBAL_TASK
    /* 创建线程，名称是 gimbal ，入口是 gimbal_thread_entry */
    gimbal_thread_handle = rt_thread_create("gimbal",
                                             gimbal_thread_entry, RT_NULL,
                                             1024,15, 10);
    /* 如果获得线程控制块，启动这个线程 */
    if (gimbal_thread_handle != RT_NULL)
        rt_thread_startup(gimbal_thread_handle);
#endif /* BSP_USING_GIMBAL_TASK */

    return RT_EOK;
}
INIT_APP_EXPORT(robot_task_init);





