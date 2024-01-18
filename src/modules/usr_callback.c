/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
*/
#include "usr_callback.h"
#include "rm_module.h"

// 将CAN1和CAN2注册的回调函数分开，避免两个总线上的相同ID冲突
#ifdef BSP_USING_CAN1
rt_err_t can1_rx_call(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg rxmsg = {0};
    uint8_t *rxbuff = rxmsg.data;

    /* 从 CAN 读取一帧数据 */
    rt_device_read(dev, 0, &rxmsg, sizeof(rxmsg));
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
#ifdef BSP_USING_DJI_MOTOR
    dji_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_HT_MOTOR
    ht_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_HT_MOTOR */
#ifdef BSP_USING_LK_MOTOR
    lk_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_LK_MOTOR */

    return RT_EOK;
}
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
rt_err_t can2_rx_call(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg rxmsg = {0};
    uint8_t *rxbuff = rxmsg.data;

    /* 从 CAN 读取一帧数据 */
    rt_device_read(dev, 0, &rxmsg, sizeof(rxmsg));
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
#ifdef BSP_USING_DJI_MOTOR
    dji_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_HT_MOTOR
    ht_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_HT_MOTOR */
#ifdef BSP_USING_LK_MOTOR
    lk_motot_rx_callback(dev, rxmsg.id, rxbuff);
#endif /* BSP_USING_LK_MOTOR */

    return RT_EOK;
}
#endif /* BSP_USING_CAN2 */
