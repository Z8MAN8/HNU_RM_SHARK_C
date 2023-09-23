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



## 使用示例

### Msg：

```c
/* Msg模块主要用于控制应用层之间的数据传递，
* 实现了发布者发布话题，订阅者订阅话题的通讯方式，
* 话题的本质是根据话题类型大小开辟的一段存储空间，
* 一定程度上避免了全局变量满天飞的情况，发布者和订阅者只需知道话题格式即可正常读写，
* 相比邮箱和消息队列等通信方式，这种方式不太优雅，仅在发布者发布消息时使用信号量进行保护，
* 相比直接使用全局变量，效率有一定程度的降低。*/

/* 首先需要在.h文件中定义话题的数据格式 */
struct msg_test{
    uint8_t id;
    uint8_t data[5];
}

/* 在发布者的.c文件中 */
publisher_t *pub;
stuct msg_test msg_p;
pub = pub_register("msg_test", sizeof(struct msg_test));
msg_p.id = 1;
for(uint8_t i = 0, i < 5, i++){
    msg_p.data[0] = i;
}
pub_push_msg(pub, &msg_p);

/* 在订阅者的.c文件中 */
subscriber_t *sub;
stuct msg_test msg_s;
sub = sub_register("msg_test", sizeof(struct msg_test));
sub_get_msg(sub, &msg_s);
```

### MAG：

这是抽象出来的磁力计一类设备，具体不同的磁力计驱动程序中需要对接 init 和 read 这些操作方法：

```c
/* 以 ist8310.c 为例 */
struct mag_ops mag = {
    .mag_init = drv_ist8310_init,
    .mag_read = ist8310_read,
};
```

在应用层中实际使用磁力计设备如下即可：

```c
static float read_data[3];
mag.mag_init("i2c1");      // 初始化 mag 设备
mag.mag_read(read_data);  // 将设备数据读取到 read_data 中
```

