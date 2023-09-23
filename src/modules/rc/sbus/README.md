# RC_SBUS

## 注意事项：

目前C型开发板处理 SBUS 接收机时，采用的方案为空闲中断 + DMA 双缓冲的方案，而且 SBUS 接收机的波特率为 100k bps，因此使用 RT-Thread 串口设备驱动不便配置。于是目前 RC_SBUS 直接**基于 HAL 库实现**，在 menuconfig 中不要使能串口3相关选项。

- `sbus_rc_init` 函数中对 DMA 和 UART3 进行设置：

  ```c
      /* DMA controller clock enable */
      __HAL_RCC_DMA1_CLK_ENABLE();
      /* DMA1_Stream1_IRQn interrupt configuration */
      HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  
      huart3.Instance = USART3;
      huart3.Init.BaudRate = 100000;
      huart3.Init.WordLength = UART_WORDLENGTH_9B;
      huart3.Init.StopBits = UART_STOPBITS_2;
      huart3.Init.Parity = UART_PARITY_EVEN;
      huart3.Init.Mode = UART_MODE_TX_RX;
      huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart3.Init.OverSampling = UART_OVERSAMPLING_16;
      HAL_UART_Init(&huart3);
  ```

- `static void rc_doub_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)` 设置 DMA 双缓冲相关；

- 在这里串口的接收回调函数就不在 user_callback 文件中实现了，而是直接在 `USART3_IRQHandler` 中实现 DMA 双缓冲切换和遥控器的数据解析；相较于电机CAN的回调函数，这样做的原因是：一条CAN总线上可能挂载多种类型的设备如电机，超级电容等，所以需要在CAN接收回调函数中对数据来源进行判断分类，但是对于串口类型，是固定连接的点对点的两个设备，因此可以将遥控器数据处理的函数直接对接注册，这样还能减少函数深度，提高效率。

- `rc_obj_t *sbus_rc_init(void)` 的返回值为指向 NOW 和 LAST 两次数据的数组起始地址，应仅在 CMD 线程中调用。

## 使用示例

应仅在 CMD 线程中调用，使用示例如下：

```c
rc_obj_t rc_data[2];   // [0]:当前数据NOW,[1]:上一次的数据LAST
rc_data = sbus_rc_init();
/* 现在就可以使用遥控器数据在 cmd 中做进一步的处理了 */
```

## TODO：

- 目前实现方法基于 HAL 库，且针对C板直接使用 UART3 及其 DMA，平台移植性较差，后续需要继续优化。
- DMA 及其中断设置放在该模块中，如果其他外设也需要使用到同一个 DMA 可能产生冲突。
- 可以增加更多通道的数据处理。
