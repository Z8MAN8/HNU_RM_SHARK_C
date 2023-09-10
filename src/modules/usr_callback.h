/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
*/

#ifndef RTTHREAD_USR_CALLBACK_H
#define RTTHREAD_USR_CALLBACK_H

#include <rtthread.h>

#ifdef BSP_USING_CAN1
rt_err_t can1_rx_call(rt_device_t dev, rt_size_t size);
#endif /* BSP_USING_CAN1 */
#ifdef BSP_USING_CAN2
rt_err_t can2_rx_call(rt_device_t dev, rt_size_t size);
#endif /* BSP_USING_CAN2 */

#endif //RTTHREAD_USR_CALLBACK_H
