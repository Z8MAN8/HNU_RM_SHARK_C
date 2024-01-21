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

#ifdef BSP_USING_DJI_MOTOR
#include "dji_motor.h"
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_DWT
#ifdef BSP_USING_HT_MOTOR
#include "ht04.h"
#endif /* BSP_USING_HT_MOTOR */
#ifdef BSP_USING_LK_MOTOR
#include "lk_motor.h"
#endif /* BSP_USING_LK_MOTOR */
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
#endif /* BSP_USING_IMU */
#ifdef BSP_USING_RC_DBUS
#include "rc_dbus.h"
#endif /* BSP_USING_RC_DBUS */
#ifdef BSP_USING_RC_KEYBOARD
#include "keyboard.h"
#endif /* BSP_USING_RC_KEYBOARD */
#ifdef BSP_USING_REFEREE
#include "BSP_CRC.h"
#include "fifo.h"
#include "Referee_system.h"
#endif /* BSP_USING_REFEREE */
#ifdef BSP_USING_LEG
#include "drv_leg.h"
#endif /* BSP_USING_LEG */

#endif /* _RM_MODULE_H */
