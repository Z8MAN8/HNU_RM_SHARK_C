# ht_motor

> TODO:
>
> - [ ] 对电机离线增加对应对声光提示；

## 总览和封装说明

lk_motor模块主要针对瓴控M系列电机，其他型号，需要自行对比电机具体使用说明。

目前仅对接多电机控制模式（具体协议查看电机手册），后续根据需求可以适配更多模式，根据电机CAN协议手册，多电机模式下，电机的ID为 1~4 ，反馈报文标识符为 0x140+ID(1~4)，控制器发送 ID 为 0x280 （更多具体设置细节需要参考官网上位机手册等 [下载中心-上海瓴控科技有限公司 (lkmotor.cn)](http://www.lkmotor.cn/Download.aspx?ClassID=21)

## 类型定义

```c
/**
 * @brief LK motor feedback
 */
typedef struct
{
    float total_angle;        // 角度为多圈角度,范围是-95.5~95.5,单位为rad
    int32_t total_round;      // 总圈数
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值，16 bit
    int16_t real_current;     // 实际转矩电流,在 -2048 ~ 2048 之间，对应 实际转矩电流范围 -33A ~ 33A
    int8_t temperature;       // 温度,C°
    float  target;            // 目标值(输出轴扭矩矩/速度/角度(单位度))
} lk_motor_measure_t;

/**
 * @brief LK intelligent motor typedef
 */
typedef struct lk_motor_object
{
    rt_device_t  can_dev;                   // 电机CAN实例
    lk_motor_measure_t measure;             // 电机测量值

    uint32_t tx_id;                         // 发送id(主发)
    uint32_t rx_id;                         // 接收id(主收)

    motor_type_e motor_type;                // 电机类型
    motor_working_type_e stop_flag;         // 启停标志

    /* 监控线程相关 */
    rt_timer_t timer;                       // 电机监控定时器

    /* 电机控制相关 */
    void *controller;            // 电机控制器
    int16_t (*control)(lk_motor_measure_t measure);   // 控制电机的接口,用户可以自定义,多电机模式下返回值为转矩电流，范围-2000~2000
} lk_motor_object_t;
```

- `lk_motor_measure_t`是电机的反馈信息；
- 使用软件定时器监控电机是否离线，若超出规定时间没有接收到反馈报文并对定时器进行更新，则判定为电机离线；
- `lk_motor_object_t` 是一个lk电机实例。一个电机实例内包含电机的反馈信息，电机的控制设置，电机控制器，电机对应的CAN实例以及电机的类型

## 外部接口

```c
/**
 * @brief 调用此函数注册一个LK电机
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return lk_motor_object_t*
 */
lk_motor_object_t *lk_motor_register(motor_config_t *config, void *control);

/**
 * @brief 该函数被motor_task调用运行在rtos上
 */
void lk_motor_control();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void lk_motor_relax(lk_motor_object_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void lk_motor_enable(lk_motor_object_t *motor);

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void lk_motot_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data);
```

- `lk_motor_register()`是用于初始化电机对象的接口，传入包括电机can配置、电机控制配置、电机控制器配置以及电机类型在内的初始化参数。**它将会返回一个电机实例指针**；

- `lk_motor_control()`是根据电机的配置计算控制值的函数。该函数在`motor_task.c`中被调用，以一定频率运行。

- `lk_motor_relax()`和`lk_motor_enable()`用于控制电机的启动和停止。当电机被设为stop的时候，不会响应任何的参考输入。

## 私有函数和变量

在.c文件内设为static的函数和变量

```c
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* 瓴控电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static lk_motor_object_t *lk_motor_obj[LK_MOTOR_CNT] = {NULL};
```

这是管理所有电机实例的入口。idx用于电机初始化。

## 使用范例

可参考 DJI 电机使用范例。
