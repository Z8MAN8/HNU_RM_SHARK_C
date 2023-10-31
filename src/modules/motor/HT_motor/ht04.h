 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-10-16      ChuShicheng     first version
 */
#ifndef _HT04_MOTOR_H
#define _HT04_MOTOR_H

// 注意：ht04电机目前于C型开发板CAN接口反序，需要调线
// 需要先向电机发送报文，电机才会反馈CAN报文

#include "motor_def.h"
#include <rtthread.h>
#include <rtdevice.h>

#define SPEED_BUFFER_SIZE 5

/* HT电机模式,初始化时自动进入CMD_MOTOR_MODE*/
typedef enum
{
    CMD_MOTOR_MODE = 0xFC,   // 使能,会响应指令
    CMD_RESET_MODE = 0xFD,   // 停止
    CMD_ZERO_POSITION = 0xFE // 将当前的位置设置为编码器零位
} ht_motor_mode_e;

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

/**
 * @brief HT04 motor feedback
 */
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
 * @brief HT intelligent motor typedef
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

#endif /* _HT04_MOTOR_H */
