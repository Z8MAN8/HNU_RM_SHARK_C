#  RM_Modules 说明文档。

RM_Modules为针对 RM 常用外设模块的函数封装。

| 外设 | 支持情况 |
| -------- | ----------------- |
| DWT  | 支持 |
| DJI_MOTOR（6020，3508，2006） | 支持 |
| ist8310 | 支持 |
| bmi088 | 支持 |



## 注意：

- 应用层调用该算法库时，先通过 menuconfig 使能相关外设模块选项，**仅包含 `rm_module.h` 即可**；
- `usr_callback` 文件中对注册到 RT-Thread 设备框架的回调函数进行统一管理；
- `motor` 目录下，除了电机驱动文件，还通过 `motor_control_task` 对工程中的电机进行稳定频率的控制；