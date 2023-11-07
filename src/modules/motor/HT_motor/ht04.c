 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-10-16      ChuShicheng     first version
 */
#include "ht04.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "usr_callback.h"

#define DBG_TAG   "ht04.motor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define HT_MOTOR_CNT 4
/* 滤波系数设置为1的时候即关闭滤波 */
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_BUFFER_SIZE 5
#define HT_SPEED_BIAS -0.0109901428f // 电机速度偏差,单位rad/s

#define P_MIN -95.5f   // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f   // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f    // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f    // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f   // N·m
#define T_MAX 18.0f

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* 海泰电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static ht_motor_object_t *ht_motor_obj[HT_MOTOR_CNT] = {NULL};

static void pack_contol_para(ht_motor_para_t para, uint8_t *buf);

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
  * @brief  converts unsigned int to float, given range and number of bits
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 设置电机模式,报文内容[0xff,0xff,0xff,0xff,0xff,0xff,0xff,cmd]
 *
 * @param cmd
 * @param motor
 */
static void motor_set_mode(ht_motor_object_t *motor, ht_motor_mode_e cmd)
{
    struct rt_can_msg msg;
    uint8_t size;

    // 填入报文
    msg.id = motor->tx_id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 0x08;
    memset(msg.data, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    msg.data[7] = (uint8_t)cmd; // 最后一位是命令id

    size = rt_device_write(motor->can_dev, 0, &msg, sizeof(msg));
    if (size == 0)
    {
        LOG_W("can bus [%s] , id 0x[%x] write data failed!", motor->can_dev->parent.name, motor->rx_id);
    }
}

/**
 * @brief 根据返回的can_object对反馈报文进行解析
 *
 * @param object 收到数据的object,通过遍历与所有电机进行对比以选择正确的实例
 */
static void motor_decode(ht_motor_object_t *motor, uint8_t *data)
{
    uint16_t var;   // 解析数据的中间变量
    uint8_t *rxbuff = data;
    ht_motor_measure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    rt_timer_start(motor->timer);  // 判定电机未离线，重置电机定时器

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_angle = measure->total_angle;
    var = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->total_angle = uint_to_float(var, P_MIN, P_MAX, 16);

    var = (uint16_t)(rxbuff[3] << 4) | (rxbuff[4] >> 4);
    measure->speed_rads = AverageFilter((uint_to_float(var, V_MIN, V_MAX, 12) - HT_SPEED_BIAS), measure->speed_buf, SPEED_BUFFER_SIZE);

    var = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->real_current = CURRENT_SMOOTH_COEF * uint_to_float(var, T_MIN, T_MAX, 12) +
                            (1 - CURRENT_SMOOTH_COEF) * measure->real_current;
}

//TODO: 添加具体的电机超时处理，如报警或停止等，以及影响机器人状态机等
/**
 * @brief 电机定时器超时回调函数
 * @param motor_ptr
 */
static void motor_lost_callback(void *motor_ptr)
{
    ht_motor_object_t *motor = (ht_motor_object_t *)motor_ptr;
//    ht_motor_relax(motor);
    LOG_W("Motor lost, can bus [%s] , id 0x[%x]", motor->can_dev->parent.name, motor->tx_id);
}

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void ht_motot_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data){
    // 找到对应的实例后再调用motor_decode进行解析
    for (size_t i = 0; i < idx; ++i)
    {   /* 详见HT04电机手册反馈报文 */
        if (ht_motor_obj[i]->can_dev == dev && ht_motor_obj[i]->tx_id == data[0])
        {
            motor_decode(ht_motor_obj[i], data);
        }
    }
}

void ht_motor_relax(ht_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void ht_motor_enable(ht_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

// 运算所有电机实例的控制器,发送控制报文
void ht_motor_control()
{
    ht_motor_object_t *motor;
    ht_motor_measure_t measure;
    ht_motor_para_t set; // 电机控制器计算得到的控制参数
    uint8_t size = 0;
    struct rt_can_msg msg;

    // 遍历所有电机实例,运行控制算法并填入报文
    for (size_t i = 0; i < idx; ++i)
    {
        motor = ht_motor_obj[i];
        measure = motor->measure;
        set = motor->control(measure); // 调用对接的电机控制器计算

        // 填入报文
        msg.id = motor->tx_id;
        msg.ide = RT_CAN_STDID;
        msg.rtr = RT_CAN_DTR;
        msg.len = 0x08;
        pack_contol_para(set, msg.data); // 将控制参数打包成报文数据帧
        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
        {
            rt_memset(msg.data, 0, sizeof(msg.data));
        }
        // 发送报文
        size = rt_device_write(motor->can_dev, 0, &msg, sizeof(msg));
        if (size == 0)
        {
            LOG_W("can bus [%s] , id 0x[%x] write data failed!", motor->can_dev->parent.name, motor->rx_id);
        }
    }
}

/**
 * @brief 电机初始化,返回一个电机实例
 * @param config 电机配置
 * @return ht_motor_object_t* 电机实例指针
 */
ht_motor_object_t *ht_motor_register(motor_config_t *config, void *control)
{
    ht_motor_object_t *object = (ht_motor_object_t *)rt_malloc(sizeof(ht_motor_object_t));
    rt_memset(object, 0, sizeof(ht_motor_object_t));

    // 对接用户配置的 motor_config
    object->motor_type = config->motor_type;             // HT04
    object->rx_id = config->rx_id;                       // 接收报文的ID(主收)
    object->tx_id = config->tx_id;                       // 发送报文的ID(主发)
    object->control = control;                           // 电机控制器执行
    object->set_mode = motor_set_mode;                   // 对接电机设置参数方法
    /* 查找 CAN 设备 */
    object->can_dev = rt_device_find(config->can_name);
    // 电机离线检测定时器相关
    object->timer = rt_timer_create("ht_motor",
                             motor_lost_callback,
                             object, 20,
                             RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(object->timer);

    ht_motor_enable(object);
    ht_motor_obj[idx++] = object;
    motor_set_mode(object, CMD_MOTOR_MODE);   // 初始化电机模式为MOTOR控制模式
    return object;
}

/**
  * @brief  封装一帧参数控制报文的数据帧
  * @param  para: 电机控制参数
  * @param  buf:  CAN数据帧
  * @retval
  */
static void pack_contol_para(ht_motor_para_t para, uint8_t *buf)
{
    uint16_t p, v, kp, kd, t;

    /* 输入参数限幅 */
    LIMIT_MIN_MAX(para.p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(para.v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(para.kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(para.kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(para.t,  T_MIN,  T_MAX);

    /* 转换float参数 */
    p = float_to_uint(para.p,     P_MIN,  P_MAX,  16);
    v = float_to_uint(para.v,     V_MIN,  V_MAX,  12);
    kp = float_to_uint(para.kp,   KP_MIN, KP_MAX, 12);
    kd = float_to_uint(para.kd,   KD_MIN, KD_MAX, 12);
    t = float_to_uint(para.t,     T_MIN,  T_MAX,  12);

    /* 将参数存入CAN数据帧 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
}

/* 预留命令接口，可用于调试 */
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
