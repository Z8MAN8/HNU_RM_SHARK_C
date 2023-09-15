#  RM_Algorithms 说明文档。

RM_Algorithms 为针对 RM 常用算法的函数封装。

| 算法 | 支持情况 |
| -------- | ----------------- |
| PID     | 支持 |
| LQR |                   |
| 卡尔曼滤波 |                   |



## 注意：

- 应用层调用该算法库时，先通过 menuconfig 使能相关算法选项，**仅包含 `rm_algorithm.h` 即可**



## 使用示例：

- ### PID：

  ```c
  /* 定义一个 PID 实例指针 */
  pid_obj_t *speed_pid;
  /* 配置一个 pid_config */
  pid_config_t chassis_speed_config = {
              .Kp = 10, // 4.5
              .Ki = 0,  // 0
              .Kd = 0,  // 0
              .IntegralLimit = 3000,
              .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
              .MaxOut = 12000,
      };
  /* 将 pid_config 注册到 PID 实例 */
  speed_pid = pid_register(&chassis_speed_config);
  /* 调用 PID 控制器计算 */
  output = pid_calculate(&chassis_speed_config, measure, ref);
  ```

  