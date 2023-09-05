/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
*/
#include "usr_callback.h"

#ifdef BSP_USING_DJI_MOTOR
#include "dji_motor.h"
#endif /* BSP_USING_DJI_MOTOR */

#ifdef BSP_USING_CAN
rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg rxmsg = {0};
    uint8_t *rxbuff = rxmsg.data;

    /* 从 CAN 读取一帧数据 */
    rt_device_read(dev, 0, &rxmsg, sizeof(rxmsg));
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
#ifdef BSP_USING_DJI_MOTOR
    dji_motot_rx_callback(rxmsg.id, rxbuff);
#endif /* BSP_USING_DJI_MOTOR */

    return RT_EOK;
}
#endif /* BSP_USING_CAN */
