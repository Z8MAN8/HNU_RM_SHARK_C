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

    return RT_EOK;
}
INIT_APP_EXPORT(robot_task_init);





