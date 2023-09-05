#  RM_Algorithms 说明文档。

RM_Algorithms 为针对 RM 常用算法的函数封装。

| 算法 | 支持情况 |
| -------- | ----------------- |
| PID     | 支持 |
| LQR |                   |
| 卡尔曼滤波 |                   |



## 注意：

- 应用层调用该算法库时，先通过 menuconfig 使能相关算法选项，**仅包含 `rm_algorithm.h` 即可**