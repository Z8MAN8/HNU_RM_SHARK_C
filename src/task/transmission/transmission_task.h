/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
*/

#ifndef RTTHREAD_TRANSMISSION_TASK_H
#define RTTHREAD_TRANSMISSION_TASK_H
#include "rtthread.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

/**
  * @brief CDC上下位机通信线程入口函数
  */
void transmission_task_entry(void* argument);

#endif // RTTHREAD_TRANSMISSION_TASK_H
