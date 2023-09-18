 /**
 * @file rm_algorithm.h
 * @author ChuShicheng
 * @author modified by neozng
 * @brief  RM电控算法库,仅被应用层调用
 * @date 2023-09-04
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-04      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include <rtthread.h>

#ifdef BSP_USING_EXAMPLE_TASK
#include "example_task.h"
#endif /* BSP_USING_EXAMPLE_TASK */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif /* BSP_USING_INS_TASK */

struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};

#endif /* _RM_TASK_H */
