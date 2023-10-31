# ht_motor

> TODO:
>
> - [ ] 对电机离线增加对应对声光提示；

## 总览和封装说明

ht_motor模块主要针对 ht04 电机，其他型号，需要自行对比电机具体使用说明。

**==设定值的单位==**

```c
/**
 * @brief 海泰电机控制参数，具体用法见说明书
 */
typedef struct ht_motor_para
{
    float p;     // 目标位置，单位为弧度(rad)
    float v;     // 目标速度，单位为 rad/s
    float kp;    // 为位置增益，单位为 N-m/rad
    float kd;    // 为速度增益，单位为 N-m*s/rad
    float t;     // 力矩，单位为 N-m
}ht_motor_para_t;
```


- `control`： 使用控制器实例进行的具体运算处理，返回值为 ht_motor_para_t 类型控制参数如：

  ```c
  static ht_motor_para_t chassis_control(dji_motor_measure_t measure){
      static ht_motor_para_t set;
      {
          set.p = 0;
          set.kp = 0;
          set.v = -1;
          set.kd = 1;
          set.t = 0;
      }
      return set;
  }
  ```

海泰电机有以下几种模式：

```
/* HT电机模式,初始化时自动进入CMD_MOTOR_MODE*/
typedef enum
{
    CMD_MOTOR_MODE = 0xFC,   // 使能,会响应指令
    CMD_RESET_MODE = 0xFD,   // 停止
    CMD_ZERO_POSITION = 0xFE // 将当前的位置设置为编码器零位
} ht_motor_mode_e;
```

电机CAN反馈报文的 data[0] 为自身 ID 号，通过其解析对应电机实例的反馈报文。

## 类型定义

```c
typedef struct
{
    float total_angle;        // 角度为多圈角度,范围是-95.5~95.5,单位为rad
    float last_angle;
    float speed_rads;         // 在 0 和 4095 之间，缩放 V MIN 和 V MAX
    float speed_buf[SPEED_BUFFER_SIZE];  // 速度缓冲区，用于滤波
    float real_current;     // 实际转矩电流,在 0 ~ 4095 之间，缩放到-40 和 40 安培，对应于峰值相电流
    float  target;            // 目标值(输出轴扭矩矩/速度/角度(单位度))
} ht_motor_measure_t;

/**
 * @brief DJI intelligent motor typedef
 */
typedef struct ht_motor_object
{
    rt_device_t  can_dev;                   // 电机CAN实例
    ht_motor_measure_t measure;             // 电机测量值

    uint32_t tx_id;                         // 发送id(主发)
    uint32_t rx_id;                         // 接收id(主收)

    motor_type_e motor_type;                // 电机类型
    motor_working_type_e stop_flag;         // 启停标志

    /* 监控线程相关 */
    rt_timer_t timer;                       // 电机监控定时器

    /* 电机控制相关 */
    void *controller;            // 电机控制器
    ht_motor_para_t (*control)(ht_motor_measure_t measure);   // 控制电机的接口 用户可以自定义,返回值为ht_motor_para_t 类型控制参数
    void (*set_mode)(struct ht_motor_object *motor, ht_motor_mode_e cmd);    // 用户可以调用改方法设置电机模式
} ht_motor_object_t;
```

- `ht_motor_measure_t`是海泰电机的反馈信息；
- 使用软件定时器监控电机是否离线，若超出规定时间没有接收到反馈报文并对定时器进行更新，则判定为电机离线；
- `ht_motor_object_t` 是一个ht电机实例。一个电机实例内包含电机的反馈信息，电机的控制设置，电机控制器，电机对应的CAN实例以及电机的类型，增加 `set_mode` api

## 外部接口

```c
/**
 * @brief 调用此函数注册一个HT04电机
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return ht_motor_object_t*
 */
ht_motor_object_t *ht_motor_register(motor_config_t *config, void *control);

/**
 * @brief 该函数被motor_task调用运行在rtos上
 */
void ht_motor_control();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void ht_motor_relax(ht_motor_object_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void ht_motor_enable(ht_motor_object_t *motor);

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void ht_motot_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data);
```

- `ht_motor_register()`是用于初始化电机对象的接口，传入包括电机can配置、电机控制配置、电机控制器配置以及电机类型在内的初始化参数。**它将会返回一个电机实例指针**；

- `ht_motor_control()`是根据电机的配置计算控制值的函数。该函数在`motor_task.c`中被调用，以一定频率运行。

- `ht_motor_relax()`和`ht_motor_enable()`用于控制电机的启动和停止。当电机被设为stop的时候，不会响应任何的参考输入。

## 私有函数和变量

在.c文件内设为static的函数和变量

```c
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* 海泰电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static ht_motor_object_t *ht_motor_obj[HT_MOTOR_CNT] = {NULL};
```

这是管理所有电机实例的入口。idx用于电机初始化。

针对海泰电机的多模式，结合 RT-Thread FinSH 工具，预留以下接口，可以通过串口终端调用以下命令，提升调试效率：

```c
static void relax(void)
{
    for (size_t i = 0; i < idx; i++)
    {
        ht_motor_obj[i]->stop_flag = MOTOR_STOP;
    }
}
MSH_CMD_EXPORT(relax, all motor relax);

static void enable(void)
{
    for (size_t i = 0; i < idx; i++)
    {
        ht_motor_obj[i]->stop_flag = MOTOR_ENALBED;
    }
}
MSH_CMD_EXPORT(enable, all motor enable);

static void motor(void){
    static ht_motor_para_t set; // 电机控制器计算得到的控制参数m
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(ht_motor_obj[i], CMD_MOTOR_MODE);
    }
}
MSH_CMD_EXPORT(motor, enter motor_mode);

static void out(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(ht_motor_obj[i], CMD_RESET_MODE);
    }
}
MSH_CMD_EXPORT(out, out motor_mode);

static void zero(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(ht_motor_obj[i], CMD_ZERO_POSITION);
    }
}
MSH_CMD_EXPORT(zero, set motor zero);
```



## 使用范例

可参考 DJI 电机使用范例，仅控制函数返回值不同。
