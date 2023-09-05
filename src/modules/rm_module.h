 /**
 * @file rm_algorithm.h
 * @author ChuShicheng
 * @author modified by neozng
 * @brief  RM电控模块库,仅被应用层调用
 * @date 2023-09-04
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-09-04      ChuShicheng     first version
 */
#ifndef _RM_MODULE_H
#define _RM_MODULE_H

#include <rtthread.h>
#include "usr_callback.h"

#ifdef BSP_USING_MOTOR
#include "motor_task.h"
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_DJI_MOTOR
#include "dji_motor.h"
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_DWT
#include "drv_dwt.h"
#endif /* BSP_USING_DWT */

#endif /* _RM_MODULE_H */
