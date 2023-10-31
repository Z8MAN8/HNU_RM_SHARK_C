 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-10-16      ChuShicheng     first version
 */
#ifndef _LK_MOTOR_H
#define _LK_MOTOR_H

#include "motor_def.h"
#include <rtthread.h>
#include <rtdevice.h>

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

#endif /* _LK_MOTOR_H */
