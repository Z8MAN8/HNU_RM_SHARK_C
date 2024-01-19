/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*                 ZhengWanshun
*                 YangShuo
*                 ChenSihan
*/
#include "rc_dbus.h"
#include "rm_config.h"
#include <stm32f4xx.h>

#define DBG_TAG           "rc.dbus"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define NOW 0
#define LAST 1
#define abs(x) ((x > 0) ? x : -x)

/* C板预留的遥控器接口（具备取反电路）为 uart3 */
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t dbus_rx_buf[2][RX_BUF_NUM];
static rc_dbus_obj_t rc_dbus_obj[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST
// TODO: 目前遥控器发送端关闭并不会检测为丢失，只有接收端异常才会判断为离线，
//       后续需要修改判断条件，预期效果是发送端关闭后判断为离线
static rt_timer_t rc_timer;  // 定时器，用于判断遥控器是否在线

/**
 * @brief 遥控器dbus数据解析
 *
 * @param rc_dbus_obj 指向dbus_rc实例的指针
 */
rt_err_t dbus_rc_decode(uint8_t *buff)
{
    /* 下面是正常遥控器数据的处理 */
    rc_dbus_obj[NOW].ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc_dbus_obj[NOW].ch1 -= 1024;
    rc_dbus_obj[NOW].ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc_dbus_obj[NOW].ch2 -= 1024;
    rc_dbus_obj[NOW].ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc_dbus_obj[NOW].ch3 -= 1024;
    rc_dbus_obj[NOW].ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc_dbus_obj[NOW].ch4 -= 1024;

    /* 防止遥控器零点有偏差 */
    if(rc_dbus_obj[NOW].ch1 <= 5 && rc_dbus_obj[NOW].ch1 >= -5)
        rc_dbus_obj[NOW].ch1 = 0;
    if(rc_dbus_obj[NOW].ch2 <= 5 && rc_dbus_obj[NOW].ch2 >= -5)
        rc_dbus_obj[NOW].ch2 = 0;
    if(rc_dbus_obj[NOW].ch3 <= 5 && rc_dbus_obj[NOW].ch3 >= -5)
        rc_dbus_obj[NOW].ch3 = 0;
    if(rc_dbus_obj[NOW].ch4 <= 5 && rc_dbus_obj[NOW].ch4 >= -5)
        rc_dbus_obj[NOW].ch4 = 0;

    /* 拨杆值获取 */
    rc_dbus_obj[NOW].sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc_dbus_obj[NOW].sw2 = (buff[5] >> 4) & 0x0003;

    /* 遥控器异常值处理，函数直接返回 */
    if ((abs(rc_dbus_obj[NOW].ch1) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch2) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch3) > RC_DBUS_MAX_VALUE) || \
      (abs(rc_dbus_obj[NOW].ch4) > RC_DBUS_MAX_VALUE))
    {
        memset(&rc_dbus_obj[NOW], 0, sizeof(rc_dbus_obj_t));
        return -RT_ERROR;
    }

    /* 鼠标移动速度获取 */
    rc_dbus_obj[NOW].mouse.x = buff[6] | (buff[7] << 8);
    rc_dbus_obj[NOW].mouse.y = buff[8] | (buff[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc_dbus_obj[NOW].mouse.l = buff[12];
    rc_dbus_obj[NOW].mouse.r = buff[13];

    /* 键盘按键键值获取 */
    rc_dbus_obj[NOW].kb.key_code = buff[14] | buff[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    rc_dbus_obj[NOW].wheel = buff[16] | buff[17] << 8;
    rc_dbus_obj[NOW].wheel -= 1024;

    rc_dbus_obj[LAST] = rc_dbus_obj[NOW];
}
/**
 * @brief 遥控器定时器超时回调函数
 */
static void rc_lost_callback(void *paramete)
{
    LOG_W("Dbus RC lost!");
}

/**
 * @brief 串口 DMA 双缓冲初始化
 * @param rx1_buf 缓冲区1
 * @param rx2_buf 缓冲区2
 * @param dma_buf_num DMA缓冲区大小
 */
static void rc_doub_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = RX_BUF_NUM;

            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == DBUS_FRAME_SIZE)
            {
                //处理遥控器数据
                dbus_rc_decode(dbus_rx_buf[0]);
                rt_timer_start(rc_timer);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = RX_BUF_NUM;

            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == DBUS_FRAME_SIZE)
            {
                //处理遥控器数据
                dbus_rc_decode(dbus_rx_buf[1]);
                rt_timer_start(rc_timer);
            }
        }
    }
}

/**
 * @brief 初始化dbus_rc
 *
 * @return rc_dbus_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_dbus_obj_t *dbus_rc_init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 100000;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_EVEN;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    rc_doub_dma_init(dbus_rx_buf[0], dbus_rx_buf[1], RX_BUF_NUM);

    // 遥控器离线检测定时器相关
    rc_timer = rt_timer_create("rc_dbus",
                               rc_lost_callback,
                               RT_NULL, 20,
                               RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(rc_timer);

    return rc_dbus_obj;
}
