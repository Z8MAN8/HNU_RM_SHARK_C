//
// Created by Administrator on 2024/1/19.
//

#include "referee_task.h"
#include "drv_gpio.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
/* --------------------------------裁判系统串口句柄 ------------------------------- */
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;
/*裁判系统线程入口*/
void referee_thread_entry(void *argument){

    /*裁判系统初始化*/
    Referee_system_Init(RX_AgreementData_Buffer0,RX_AgreementData_Buffer1,Agreement_RX_BUF_NUM);
    graphic_data_struct_t TEST_data;
    /*裁判系统数据解包*/
    for(;;) {
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前缓冲区是0，解包0缓冲区，否则解包1缓冲区
        {
            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
        } else {
            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
        }
    }
}

void USART6_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)//如果串口中断开启
    {
        static uint16_t this_time_rx_len = 0;
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //清除空闲中断
        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前的缓冲区是缓冲区0
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            //将这1帧数据放入fifo0
            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer0,this_time_rx_len);
        }
        else //如果当前的缓冲区是缓冲区1
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            //osSemaphoreRelease(RefereeRxOKHandle);  //释放信号量
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区0
            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            fifo_s_puts(&RX_AgreementData_FIFO,(char *)RX_AgreementData_Buffer1,this_time_rx_len);
        }
    }
    HAL_UART_IRQHandler(&huart6);
}