/*
* Change Logs:
* Date            Author          Notes
* 2024-01-19     ChenSihan    first version
*
*/
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
    /*用户3pin串口初始化*/
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    /* HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);*/
    huart6.Instance=USART6;
    huart6.Init.BaudRate=115200;
    huart6.Init.WordLength=UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity=UART_PARITY_NONE;
    huart6.Init.Mode=UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart6.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);

    /*裁判系统初始化*/
    Referee_system_Init(RX_AgreementData_Buffer0,RX_AgreementData_Buffer1,Agreement_RX_BUF_NUM);

    /*裁判系统数据解包*/
    while(1)
    {
        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == 0U) //如果当前缓冲区是0，解包0缓冲区，否则解包1缓冲区
        {
            Referee_Data_Unpack(RX_AgreementData_Buffer0, &Referee_Data_header, &Referee_Data);
        }
        else
        {
            Referee_Data_Unpack(RX_AgreementData_Buffer1, &Referee_Data_header, &Referee_Data);
        }
        rt_thread_mdelay(1);
    }
}

void USART6_IRQHandler(void)
{
    if (huart6.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(&huart6);      //清除空闲中断

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) //如果当前的缓冲区是缓冲区0
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            //将这1帧数据放入fifo0
            fifo_s_puts(&RX_AgreementData_FIFO, (char *) RX_AgreementData_Buffer0, this_time_rx_len);
        }
        else //如果当前的缓冲区是缓冲区1
        {
            //计算这一帧接收的数据的长度
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = Agreement_RX_BUF_NUM -hdma_usart6_rx.Instance->NDTR;
            //osSemaphoreRelease(RefereeRxOKHandle);  //释放信号量
            //重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = Agreement_RX_BUF_NUM;
            //把缓冲区设置成缓冲区0
            hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            fifo_s_puts(&RX_AgreementData_FIFO, (char *) RX_AgreementData_Buffer1, this_time_rx_len);

        }
    }
    HAL_UART_IRQHandler(&huart6);
}