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
#endif /* BSP_USING_PS_MSG */
#ifdef BSP_USING_PS_MSG
#include "drv_msg.h"
#endif /* BSP_USING_IST8310 */
#ifdef BSP_USING_MAG
#include "mag.h"
#endif /* BSP_USING_IST8310 */
#ifdef BSP_USING_IMU
#include "imu.h"
#endif /* BSP_USING_RC_SBUS */
#ifdef BSP_USING_RC_SBUS
#include "rc_sbus.h"
#endif /* BSP_USING_RC_SBUS */
#ifdef BSP_USING_BUZZER
#include "buzzer.h"
#endif /* BSP_USING_BUZZER */

#endif /* _RM_MODULE_H */
