/**
 * @file dji_motor.h
 * @author neozng
 * @version 0.2
 * @date 2022-11-01
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "motor_def.h"
#include <rtthread.h>
#include <rtdevice.h>

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
    int16_t  speed_rpm;       // 电机的转速值
    int16_t real_current;     // 实际转矩电流
    uint8_t temperature;      // Celsius
} dji_motor_measure_t;

/**
 * @brief DJI intelligent motor typedef
 */
typedef struct dji_motor_object
{
    rt_device_t  can_dev;                   // 电机CAN实例
    dji_motor_measure_t measure;            // 电机测量值

    uint32_t tx_id;                         // 发送id(主发)
    uint32_t rx_id;                         // 接收id(主收)
    /* 分组发送设置 */
    uint8_t send_group;                     // 同一帧报文分组
    uint8_t message_num;                    // 一帧报文中位置

    motor_type_e motor_type;                // 电机类型
    motor_working_type_e stop_flag;         // 启停标志

    /* 监控线程相关 */
    rt_timer_t timer;                       // 电机监控定时器

    /* 电机控制相关 */
    void *controller;            // 电机控制器
    int16_t (*control)(dji_motor_measure_t measure);   // 控制电机的接口 用户可以自定义,返回值为16位的电压或电流值
} dji_motor_object_t;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小dji_motor_control()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return dji_motor_object_t*
 */
dji_motor_object_t *dji_motor_register(motor_config_t *config, void *control);

/**
 * @brief 该函数被motor_task调用运行在rtos上
 */
void dji_motor_control();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void dji_motor_relax(dji_motor_object_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void dji_motor_enable(dji_motor_object_t *motor);

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
void dji_motot_rx_callback(rt_device_t dev, uint32_t id, uint8_t *data);

#endif /* _DJI_MOTOR_H */
