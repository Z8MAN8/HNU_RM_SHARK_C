/*
* Change Logs:
* Date            Author          Notes
* 2023-09-25      ChuShicheng     first version
*/
#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include <rtthread.h>

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_STOP,          //底盘停止
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY,           //底盘飞坡模式
    CHASSIS_AUTO           //底盘自动模式
} chassis_mode_e;

void chassis_thread_entry(void *argument);

#endif /* _CHASSIS_TASK_H */
