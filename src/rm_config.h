 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 */
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

#define CPU_FREQUENCY 168     /* CPU主频(mHZ) */

/* 底盘和云台分别对应的 can 设备名称 */
#define CAN_CHASSIS    "can1"
#define CAN_GIMBAL     "can2"

/* 磁力计所挂载的 i2c 设备名称(软件i2c) */
#define I2C_MAG        "i2c1"    //"Notice: PA8 --> 8; PC9 --> 41"

/* 陀螺仪所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_GYRO       "spi1"
#define SPI_GYRO_CS    16
/* 加速度计所挂载的 SPI 设备名称及 CS 引脚 */
#define SPI_ACC        "spi1"
#define SPI_ACC_CS     4

/* 蜂鸣器所挂载的 PWM 设备名称 */
#define PWM_BUZZER     "pwm4"
#define BUZZER_CHANNEL 3

/* 遥控器所挂载的 usart 设备名称 */
#define USART_RC       "uart3"

/* ---------------------------------- 遥控器相关 --------------------------------- */
#define RC_MAX_VALUE      784.0f  /* 遥控器通道最大值 */

#define RC_RATIO          0.0009f

/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f

/* ---------------------------------- 底盘相关 ---------------------------------- */
/* 底盘轮距(mm) */
#define WHEELTRACK        300
/* 底盘轴距(mm) */
#define WHEELBASE         388
/* 底盘轮子周长(mm) */
#define WHEEL_PERIMETER   481

#define LENGTH_A 278 //底盘长的一半(mm)
#define LENGTH_B 294 //底盘宽的一半(mm)

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转 */
#define MAX_WHEEL_RPM        9000   //8347rpm = 3500mm/s

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 5000
#define MAX_CHASSIS_VY_SPEED 5000
/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 8

/* --------------------------------- 底盘PID参数 -------------------------------- */
/* 电机速度环 */
#define CHASSIS_KP_V_MOTOR              6
#define CHASSIS_KI_V_MOTOR              15
#define CHASSIS_KD_V_MOTOR              0
#define CHASSIS_INTEGRAL_V_MOTOR        8000
#define CHASSIS_MAX_V_MOTOR             16000
// TODO: 参数待整定
/* 跟随云台PID */
#define CHASSIS_KP_V_FOLLOW             0.05
#define CHASSIS_KI_V_FOLLOW             0
#define CHASSIS_KD_V_FOLLOW             0
#define CHASSIS_INTEGRAL_V_FOLLOW       300
#define CHASSIS_MAX_V_FOLLOW            1500

/* ---------------------------------- 云台相关 ---------------------------------- */
#define YAW_MOTOR_ID     0x207
#define PITCH_MOTOR_ID   0x208

#define CENTER_ECD_YAW   7837         //云台yaw轴编码器归中值
#define CENTER_ECD_PITCH 1420         //云台pitch轴编码器归中值

/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        31.5f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -32.9f

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             5000
#define YAW_KI_V_IMU             200
#define YAW_KD_V_IMU             10
#define YAW_INTEGRAL_V_IMU       1000
#define YAW_MAX_V_IMU            30000
/* imu角度环 */
#define YAW_KP_A_IMU             0.35f
#define YAW_KI_A_IMU             0
#define YAW_KD_A_IMU             0.001f
#define YAW_INTEGRAL_A_IMU       5
#define YAW_MAX_A_IMU            25
/* auto速度环 */
#define YAW_KP_V_AUTO            0
#define YAW_KI_V_AUTO            0
#define YAW_KD_V_AUTO            0
#define YAW_INTEGRAL_V_AUTO      0
#define YAW_MAX_V_AUTO           0
/* auto角度环 */
#define YAW_KP_A_AUTO            0
#define YAW_KI_A_AUTO            0
#define YAW_KD_A_AUTO            0
#define YAW_INTEGRAL_A_AUTO      0
#define YAW_MAX_A_AUTO           0

/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU           4250
#define PITCH_KI_V_IMU           1000
#define PITCH_KD_V_IMU           3
#define PITCH_INTEGRAL_V_IMU     1500
#define PITCH_MAX_V_IMU          20000
/* imu角度环 */
#define PITCH_KP_A_IMU           0.5f
#define PITCH_KI_A_IMU           0.0f
#define PITCH_KD_A_IMU           0.005f
#define PITCH_INTEGRAL_A_IMU     0.2f
#define PITCH_MAX_A_IMU          20
/* auto速度环 */
#define PITCH_KP_V_AUTO          0
#define PITCH_KI_V_AUTO          0
#define PITCH_KD_V_AUTO          0
#define PITCH_INTEGRAL_V_AUTO    0
#define PITCH_MAX_V_AUTO         0
/* auto角度环 */
#define PITCH_KP_A_AUTO          0
#define PITCH_KI_A_AUTO          0
#define PITCH_KD_A_AUTO          0
#define PITCH_INTEGRAL_A_AUTO    0
#define PITCH_MAX_A_AUTO         0

#endif /* _RM_CONFIG_H */
