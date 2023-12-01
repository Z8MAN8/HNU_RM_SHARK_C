/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
*/

#include "transmission_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static void trans_sub_pull(void);
static void trans_sub_init(void);
/**
 * @brief trans 线程中所有订阅者初始化（如有其它数据需求可在其中添加）
 */
static void trans_sub_init(void)
{
}

/**
 * @brief trans 线程中所有订阅者获取更新话题（如有其它数据需求可在其中添加）
 */
static void trans_sub_pull(void)
{
}

/* --------------------------------- 通讯线程入口 --------------------------------- */
static float trans_dt;

void transmission_task_entry(void* argument)
{
    static float trans_start;

    /*订阅数据初始化*/
    trans_sub_init();

    LOG_I("Transmission Task Start");
    while (1)
    {
        trans_start = dwt_get_time_ms();
        /*订阅数据更新*/
        trans_sub_pull();

/*--------------------------------------------------具体需要发送的数据--------------------------------- */

/*--------------------------------------------------具体需要发送的数据---------------------------------*/

        /* 用于调试监测线程调度使用 */
        trans_dt = dwt_get_time_ms() - trans_start;
        if (trans_dt > 1)
            LOG_E("Transmission Task is being DELAY! dt = [%f]", &trans_dt);
        rt_thread_delay(1);
    }

}
