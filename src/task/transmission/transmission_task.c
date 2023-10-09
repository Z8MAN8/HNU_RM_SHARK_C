//
// Created by DAMIAN-CHEN on 2023/10/7.
//

#include "transmission_task.h"

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct ins_msg ins_data;
static subscriber_t  *sub_ins;

static void transmission_sub_pull(void);
static void transmission_sub_init(void);
/**
 * @brief transmission 线程中所有订阅者初始化（如有其它数据需求可在其中添加）
 */
static void transmission_sub_init(void)
{
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}

/**
 * @brief transmission 线程中所有订阅者获取更新话题（如有其它数据需求可在其中添加）
 */
static void transmission_sub_pull(void)
{
    sub_get_msg(sub_ins, &ins_data);
}

void transmission_task_entry(void* argument)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    uint32_t *gimbal_rpy;
    rt_device_t dev_vcom = RT_NULL;
    dev_vcom = rt_device_find("vcom");
    if (dev_vcom)
        rt_device_open(dev_vcom, RT_DEVICE_FLAG_RDWR);

    transmission_sub_init();
    while (1)
    {
        transmission_sub_pull();
        /* USB发送角度帧 */
        rpy_tx_buffer[0] = 0;
        *gimbal_rpy = (int32_t)ins_data.yaw*1000;
        rpy_tx_buffer[1] = *gimbal_rpy;
        rpy_tx_buffer[2] = *gimbal_rpy >> 8;
        rpy_tx_buffer[3] = *gimbal_rpy >> 16;
        rpy_tx_buffer[4] = *gimbal_rpy >> 24;

        *gimbal_rpy = (int32_t)ins_data.pitch*1000;
        rpy_tx_buffer[5] = *gimbal_rpy;
        rpy_tx_buffer[6] = *gimbal_rpy >> 8;
        rpy_tx_buffer[7] = *gimbal_rpy >> 16;
        rpy_tx_buffer[8] = *gimbal_rpy >> 24;

        *gimbal_rpy = (int32_t)ins_data.roll*1000;
        rpy_tx_buffer[9] = *gimbal_rpy;
        rpy_tx_buffer[10] = *gimbal_rpy >> 8;
        rpy_tx_buffer[11] = *gimbal_rpy >> 16;
        rpy_tx_buffer[12] = *gimbal_rpy >> 24;
        // 这里是发送数据，云台接收数据在 `usbd_cdc_if.c` 中的**回调函数** CDC_Receive_FS() 里进行处理。
        Add_Frame_To_Upper(GIMBAL, rpy_tx_buffer);
        rt_device_write(dev_vcom, 0, (uint8_t*)&rpy_tx_data, rt_strlen((uint8_t*)&rpy_tx_data));
        rt_thread_delay(1);
    }

}




void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf){
    switch (send_mode) {
        case GIMBAL:{
            rpy_tx_data.HEAD = 0XFF;
            rpy_tx_data.D_ADDR = MAINFLOD;
            rpy_tx_data.ID = GIMBAL;
            rpy_tx_data.LEN = FRAME_RPY_LEN;
            memcpy(&rpy_tx_data.DATA, data_buf, sizeof(rpy_tx_data.DATA));

            /* 将 odom 帧先转存到中转帧中做数据校验计算 */
            memcpy(&upper_tx_data, &rpy_tx_data, sizeof(rpy_tx_data));
            rpy_tx_data.SC = (uint8_t)Sumcheck_Cal(upper_tx_data) >> 8;//截取allcheck的高八位，为实际计算出的和校验的低八位
            rpy_tx_data.AC = (uint8_t)Sumcheck_Cal(upper_tx_data);//截取allcheck的低八位，为实际计算出的附加校验的低八位
            memset(&upper_tx_data, 0, sizeof(upper_tx_data));
        }break;
        default:break;
    }
}
//TODO: 校验结果不对

uint16_t Sumcheck_Cal(BCPFrameTypeDef frame){
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint16_t allcheck = 0;

    sumcheck += frame.HEAD;
    addcheck += sumcheck;
    sumcheck += frame.D_ADDR;
    addcheck += sumcheck;
    sumcheck += frame.ID;
    addcheck += sumcheck;
    sumcheck += frame.LEN;
    addcheck += sumcheck;

    for(int i = 0; i<frame.LEN; i++){
        sumcheck += frame.DATA[i];
        addcheck += sumcheck;
    }
    allcheck = (uint16_t)(sumcheck << 8 | addcheck);//和校验的低八位为allcheck的高八位，附加校验的低八位放在allcheck的低八位的位置
    return allcheck;
}


