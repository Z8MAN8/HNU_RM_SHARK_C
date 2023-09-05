# dji_motor

<p align='right'>neozng1@hnu.edu.cn</p>

> TODO:
>
> - [ ] 对电机离线增加对应对声光提示；

---

> 单条总线挂载电机数量有限（实测1kHZ极限挂载7个），挂载数量过多容易出现帧错误和仲裁失败的情况。

## 总览和封装说明

> 如果你不需要理解该模块的工作原理，你只需要查看这一小节。

dji_motor模块对DJI智能电机，包括M2006，M3508以及GM6020进行了详尽的封装。你不再需要关心CAN报文的发送和接收解析。

**==设定值的单位==**

1. ==GM6020的输入设定为**力矩**，待测量（-30000~30000）==

   ==M3508的输入设定为-20A~20A （-16384~16384）==

   ==M2006的输入设定为-10A~10A （-10000~10000）==

要获得一个电机，请通过`dji_motor_register()`并传入一些参数，他就会返回一个电机的指针。你也不再需要查看这些电机和电调的说明书，**只需要设置其电机id**（6020为拨码开关值，2006和3508为电调的闪动次数），该模块会自动为你计算CAN发送和接收ID并搞定所有硬件层的琐事。

**注意：**6020的id 1-4和2006/3508的id 5-8会发生冲突（即1!5,2!6,3!7,4!8），不能同时注册在同一总线。

初始化电机时，你需要传入的参数包括：

- **电机挂载的CAN总线设置**，CAN1 or CAN2，以及电机的id，使用`can_object_config_s`封装，只需要设置这两个参数:

  ```c
  CAN_HandleTypeDef *can_handle;
  uint32_t tx_id; // tx_id设置为电机id,不需要查说明书计算，直接为电调的闪动次数或拨码开关值，为1-8
  ```

- **电机类型**，使用`motor_type_e`：

  ```c
  GM6020 = 0
  M3508  = 1
  M2006  = 2
  ```

- **电机控制设置**

  - 是否反转

    ```c
    MOTOR_DIRECTION_NORMAL 
    MOTOR_DIRECTION_REVERSE
    ```
    


---

## 代码结构

.h文件内包括了外部接口和类型定义,以及模块对应的宏。c文件内为私有函数和外部接口的定义。

motor_def.h内包含了一些电机通用的定义。

## 类型定义

```c
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

/**
 * @brief DJI motor feedback
 */
typedef struct
{
    /* 以下是处理得出的数据 */
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    float total_angle;        // 总角度,注意方向
    int32_t total_round;      // 总圈数,注意方向
    float  target;            // 目标值(输出轴扭矩矩/速度/角度(单位度))

    /* 以下是电调直接回传的数据 */
    uint16_t ecd;             // 0-8191
    uint16_t last_ecd;        // 上一次读取的编码器值
    int16_t  speed_rpm;       //电机的转速值
    int16_t real_current;     // 实际转矩电流
    uint8_t temperature;      // Celsius
} dji_motor_measure_t;

/**
 * @brief DJI intelligent motor typedef
 */
typedef struct
{
    dji_motor_measure_t measure;            // 电机测量值
    can_obj_t *can_dev;            // 电机CAN实例

    /* 分组发送设置 */
    uint8_t send_group;                     // 同一帧报文分组
    uint8_t message_num;                    // 一帧报文中位置

    motor_type_e motor_type;                // 电机类型
    motor_working_type_e stop_flag;         // 启停标志

    /* 监控线程相关 */
    rt_timer_t timer;                       // 电机监控定时器

// TODO: 在用户配置文件中对接，考虑配置文件设为.c和.h文件,.c文件中存放相关对接函数的具体实现，.h文件中进行config结构体的配置，意图用户最终在user_config.c/.h中进行开发即可
    /* 电机控制相关 */
    void *motor_settings;      // 电机设置
    void *motor_controller;    // 电机控制器
    void (*control)(dji_motor_object_t *motor);   // 控制电机的接口 用户可以自定义
} dji_motor_object_t;
```

- `SPEED_SMOOTH_COEF`和`CURRENT_SMOOTH_COEF`是电机反馈的电流和速度数据低通滤波器惯性系数，数值越小平滑效果越大，但滞后也越大。设定时不应当低于推荐值。
- `dji_motor_measure`是DJI电机的反馈信息，包括当前编码器值、上次测量编码器值、速度、电流、温度、总圈数和单圈角度，目标值。
- 使用软件定时器监控电机是否离线，若超出规定时间没有接收到反馈报文并对定时器进行更新，则判定为电机离线。
- `dji_motor_object`是一个DJI电机实例。一个电机实例内包含电机的反馈信息，电机的控制设置，电机控制器，电机对应的CAN实例以及电机的类型；由于DJI电机支持**一帧报文控制至多4个电机**，该结构体还包含了用于给电机分组发送进行特殊处理的`send_group`和`message_num`（具体实现细节参考`motor_send_grouping()`函数）。

## 外部接口

```c
dji_motor_object *dji_motor_register(can_object_config config,
                                 Motor_Control_Setting_s motor_setting,
                                 Motor_Controller_Init_s controller_init,
                                 motor_type_e type);

void dji_motor_control();

void dji_motor_stop(dji_motor_object *motor);

void dji_motor_enable(dji_motor_object *motor);
```

- `dji_motor_register()`是用于初始化电机对象的接口，传入包括电机can配置、电机控制配置、电机控制器配置以及电机类型在内的初始化参数。**它将会返回一个电机实例指针**，你应当在应用层保存这个指针，这样才能操控这个电机。

- `dji_motor_control()`是根据电机的配置计算控制值的函数。该函数在`motor_task.c`中被调用，应当在freeRTOS中以一定频率运行。此函数为PID的计算进行了彻底的封装，要修改电机的参考输入，请在app层的应用中调用`DJIMotorSetRef()`。

  该函数的具体实现请参照代码，注释已经较为清晰。流程大致为：

  1. 根据电机的初始化控制配置，计算各个控制闭环
  2. 根据反转标志位，确定是否将输出反转
  3. 根据每个电机的发送分组，将最终输出值填入对应的分组buff
  4. 检查每一个分组，若该分组有电机，发送报文
  
- `dji_motor_stop()`和`dji_motor_enable()`用于控制电机的启动和停止。当电机被设为stop的时候，不会响应任何的参考输入。

- `DJIMotorOuterLoop()`用于修改电机的外部闭环类型，即电机的真实闭环目标。

## 私有函数和变量

在.c文件内设为static的函数和变量

```c
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
static dji_motor_object *dji_motor_info[DJI_MOTOR_CNT] = {NULL};
```

这是管理所有电机实例的入口。idx用于电机初始化。

```c
#define ECD_ANGLE_COEF_DJI 3.835e-4 // ecd/8192*pi
```

这两个宏用于在电机反馈信息中的多圈角度计算，将编码器的0~8192转化为角度表示。

```c
/* @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_object专门负责发送
 *        该变量将在 dji_motor_control() 中使用,分组在 motor_send_grouping()中进行
 *
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [0]:0x1FF,[1]:0x200,[2]:0x2FF */
static can_object send_msg[6] =
{
        [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
  ...
        ...
};

static uint8_t sender_enable_flag[6] = {0};
```

- 这些是电机分组发送所需的变量。注册电机时，会根据挂载的总线以及发送id，将电机分组。在CAN发送电机控制信息的时候，根据`send_msg[]`保存的分组进行发送，而不会使用电机实例自带的`can_object`。
- DJI电机共有3种分组，分别为0x1FF,0x200,0x2FF。注册电机的时候，`motor_send_grouping()`函数会根据发送id计算出CAN的`tx_id`（即上述三个中的一个）和`rx_id`。然后为电机实例分配用于指示其在`send_msg[]`中的编号的 `send_group`和其在该发送组中的位置`message_num`（一帧报文可以发送四条控制指令，`message_num`会指定电机是这四个中的哪一个）。具体的分配请查看`motor_send_grouping()`的定义。
- 当某一个分组有电机注册时，该分组的索引将会在`sender_enable_flag`[]中被置1，这样，就可以避免发送没有电机注册的报文，防止总线拥塞。具体的，在`decode_dji_motor()`中，该函数会查看`sender_enable_flag[]`的每一个位置，确定这一组是否有电机被注册，若有则发送`send_msg[]`中对应位置的`tx_buff`。

```c
static void motor_send_grouping(can_object_config *config)

static void decode_dji_motor(can_object *object)
```

- `IDcrash_Handler()`在电机id发生冲突的时候会被`motor_send_grouping()`调用，陷入死循环之中，并把冲突的id保存在函数里。这样就可以通过debug确定是否发生冲突以及冲突的编号。

- `motor_send_grouping()`被`dji_motor_register()`调用，他将会根据电机id计算出CAN的发送和接收ID，并根据发送ID对电机进行分组。

- `decode_dji_motor()`是解析电机反馈报文的函数，在`dji_motor_register()`中会将其注册到该电机实例对应的`can_object`中（即`can_object`的`can_module_callback()`）。这样，当该电机的反馈报文到达时，`bsp_can.c`中的回调函数会调用解包函数进行反馈数据解析。

  该函数还会对电流和速度反馈值进行滤波，消除高频噪声；同时计算多圈角度和单圈绝对角度。
  
  **电机反馈的电流值为说明书中的映射值，需转换为实际值。**
  
  **反馈的速度单位是rpm（转每分钟），转换为角度每秒。**
  
  **反馈的位置是编码器值（0~8191），转换为角度。**

## 使用范例

```c
//初始化设置
motor_config_t config = {
  .motor_type = GM6020,
  .can_init_config = {
   .can_handle = &hcan1,
   .tx_id = 6
        },
  .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP, 
            .speed_feedback_source = MOTOR_FEED, 
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
  .controller_param_init_config = {
            .angle_PID = {
                .Improve = 0, 
                .Kp = 1, 
                .Ki = 0, 
                .Kd = 0, 
                .DeadBand = 0, 
                .MaxOut = 4000}, 
            .speed_PID = {
                .Improve = 0, 
                .Kp = 1, 
                .Ki = 0, 
                .Kd = 0, 
                .DeadBand = 0, 
                .MaxOut = 4000
            }
        }
};
//注册电机并保存实例指针
dji_motor_object *djimotor = dji_motor_register(&config);
```

然后在任务中修改电机设定值即可实现控制：

```
DJIMotorSetRef(djimotor, 10);
```

前提是已经将`dji_motor_control()`放入实时系统任务当中或以一定d。你也可以单独执行`dji_motor_control()`。
