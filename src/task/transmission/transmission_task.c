/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
* 2023-12-09      YangShuo     USB虚拟串口
*/

#include "transmission_task.h"
#include "drv_gpio.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct gimbal_cmd_msg gim_cmd;
static struct ins_msg ins_data;
static struct gimbal_fdb_msg gim_fdb;
static struct trans_fdb_msg trans_fdb;
/*------------------------------传输数据相关 --------------------------------- */
#define RECV_BUFFER_SIZE 32  // 接收环形缓冲区大小
rt_uint8_t r_buffer[RECV_BUFFER_SIZE];  // 接收环形缓冲区
struct rt_ringbuffer receive_buffer ; // 环形缓冲区对象控制块指针
rt_uint8_t buf[31] = {0};
RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
RpyTypeDef rpy_rx_data; //接收解析结构体
/* ---------------------------------usb虚拟串口数据相关 --------------------------------- */
int USB_DP_PIN = GET_PIN(A,12);//USB PD引脚
static rt_device_t vs_port = RT_NULL;
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
static struct rt_semaphore rx_sem;    /* 用于接收消息的信号量 */
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static publisher_t *pub_trans;
static subscriber_t *sub_cmd,*sub_ins,*sub_gim;
static void trans_sub_pull(void);
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_pub_init(void);

/**
 * @brief trans 线程中所有订阅者初始化（如有其它数据需求可在其中添加）
 */
static void trans_sub_init(void)
{
    sub_cmd = sub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));

}

/**
 * @brief trans 线程中所有订阅者获取更新话题（如有其它数据需求可在其中添加）
 */
static void trans_sub_pull(void)
{
    sub_get_msg(sub_cmd, &gim_cmd);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_gim, &gim_fdb);
}

/**
 * @brief cmd 线程中所有发布者初始化
 */
static void trans_pub_init(void)
{
    pub_trans = pub_register("trans_fdb",sizeof(struct trans_fdb_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void trans_pub_push(void)
{
    pub_push_msg(pub_trans,&trans_fdb);
}

void transmission_task_entry(void* argument)
{
    static float trans_dt;
    static float trans_start;

    /*订阅数据初始化*/
    trans_sub_init();
    /*发布数据初始化*/
    trans_pub_init();
    /*软件模拟USB插拔*/
    rt_pin_write(USB_DP_PIN,PIN_LOW);
    /* step1：查找名为 "vcom" 的虚拟串口设备*/
    vs_port = rt_device_find("vcom");
    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_921600;        //修改波特率为 921600
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 1024;                   //修改接收缓冲区 buff size 为 1024
    config.parity    = PARITY_NONE;           //无奇偶校验位
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(vs_port, RT_DEVICE_CTRL_CONFIG,&config);
    /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备*/
    if (vs_port)
        rt_device_open(vs_port, RT_DEVICE_FLAG_INT_RX);
    /*环形缓冲区初始化*/
    rt_ringbuffer_init(&receive_buffer, r_buffer, RECV_BUFFER_SIZE);
    /*信号量初始化*/
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(vs_port, usb_input);
    LOG_I("Transmission Task Start");
    while (1)
    {
        trans_start = dwt_get_time_ms();
        /*订阅数据更新*/
        trans_sub_pull();
        /* 发布数据更新 */
        trans_pub_push();
/*--------------------------------------------------具体需要发送的数据--------------------------------- */
        SendData(rpy_tx_data);
        //Getdata();
/*--------------------------------------------------具体需要发送的数据---------------------------------*/
        /* 用于调试监测线程调度使用 */
        trans_dt = dwt_get_time_ms() - trans_start;
        if (trans_dt > 1)
            LOG_E("Transmission Task is being DELAY! dt = [%f]", &trans_dt);
        rt_thread_mdelay(1);
    }
}

void SendData(RpyTypeDef data_r)
{
    /*填充数据*/
    pack_Rpy(&data_r, (ins_data.yaw - gim_fdb.yaw_offset_angle), (ins_data.pitch - gim_fdb.pit_offset_angle), ins_data.roll);
    Check_Rpy(&data_r);

    rt_device_write(vs_port, 0, (uint8_t*)&data_r, sizeof(data_r));
}

void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch,float roll)
{


    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = roll *1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer,13);

    frame->LEN = FRAME_RPY_LEN;
}

void Check_Rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

// 接收数据回调函数
static rt_err_t usb_input(rt_device_t dev, rt_size_t size)
{
    memset(buf, 0, sizeof(buf));
    // 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量
    rt_sem_release(&rx_sem);

    // 从串口读取数据并保存到环形接收缓冲区
    rt_uint32_t rx_length;
    while ((rx_length = rt_device_read(vs_port, 0, buf, sizeof(buf))) > 0)
    {
        // 将接收到的数据放入环形缓冲区
        rt_ringbuffer_put_force(&receive_buffer, buf, rx_length);
    }
    rt_uint8_t frame_rx[sizeof(RpyTypeDef)]={0};
    rt_ringbuffer_get(&receive_buffer, frame_rx, sizeof(frame_rx));
    if(*(uint8_t*)frame_rx==0xFF)
    {
        memcpy(&rpy_rx_data,&frame_rx,sizeof(rpy_rx_data));
        switch (rpy_rx_data.ID) {
            case GIMBAL:{
                if (rpy_rx_data.DATA[0]) {//相对角度控制
                    trans_fdb.yaw = (*(int32_t*)&rpy_rx_data.DATA[1] / 1000.0);
                    trans_fdb.pitch = (*(int32_t*)&rpy_rx_data.DATA[5] / 1000.0);
                }
            }break;
        }
        memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
    }
    return RT_EOK;
}
void Getdata()
{
    rt_uint8_t frame_rx[sizeof(RpyTypeDef)]={0};
    rt_ringbuffer_get(&receive_buffer, frame_rx, sizeof(frame_rx));
    if(*(uint8_t*)frame_rx==0xFF)
    {
        memcpy(&rpy_rx_data,&frame_rx,sizeof(rpy_rx_data));
        switch (rpy_rx_data.ID) {
            case GIMBAL:{
                if (rpy_rx_data.DATA[0]) {//相对角度控制
                    trans_fdb.yaw = (*(int32_t*)&rpy_rx_data.DATA[1] / 1000.0);
                    trans_fdb.pitch = (*(int32_t*)&rpy_rx_data.DATA[5] / 1000.0);
                }
            }break;
        }
        memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
    }

}