/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     first version
* 2023-12-09      YangShuo     USB虚拟串口
*/

#ifndef RTTHREAD_TRANSMISSION_TASK_H
#define RTTHREAD_TRANSMISSION_TASK_H
#include "rtthread.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

/* BCP通讯协议相关 */
//TODO: 考虑不同帧长的情况
#define FRAME_MAX_LEN 36        /* 通讯帧的最大长度 */
#define FRAME_XYA_LEN 6         /* 速度控制方式数据长度 */
#define FRAME_AUTO_LEN 22         /* 自瞄发送方式控制数据长度 */
#define FRAME_RPY_LEN 25         /* 欧拉角rpy方式控制数据长度 */
#define FRAME_ODOM_LEN 36       /* 里程计控制方式数据长度 */
#define FRAME_CTRL_LEN 24       /* 角/线速度控制方式数据长度 */
#define FRAME_SHOOT_LEN 7       /* 发射机构数据长度 */
#define FRAME_IMU_LEN 24        /* imu控制方式数据长度 */
/* 目标地址表 */
#define BROADCAST   0x00        /* 广播 */
#define MAINFLOD    0x01        /* 上位机 */
#define SENTRY_UP   0x02        /* 哨兵机器人上云台 */
#define SENTRY_DOWN 0x03        /* 哨兵机器人下云台 */
#define INFANTRY    0x04        /* 步兵机器人 */
#define ENGINEER    0x05        /* 工程机器人 */
#define HERO        0x06        /* 英雄机器人 */
#define AIR         0x07        /* 空中机器人 */
#define RADAR       0x08        /* 雷达站 */
#define GATHER      0x09        /* 视觉采集台 */
#define STANDARD    0x10        /* AI机器人/全自动步兵机器人 */
/* 功能码表 */
#define CHASSIS                 0x10        /* 速度方式控制 */
#define CHASSIS_ODOM            0x11        /* 里程计方式控制 */
#define CHASSIS_CTRL            0x12        /* 角/线速度方式控制 */
#define CHASSIS_IMU             0x13        /* 底盘imu数据 */
#define GIMBAL                  0x20        /* 欧拉角rpy方式控制 */
#define GAME_STATUS             0x30        /* 比赛类型数据*/
#define ROBOT_HP                0x31        /* 机器人血量数据 */
#define ICRA_BUFF_DEBUFF_ZONE   0x32        /* 增益区数据 */
#define GAME_MODE               0x33        /* 机器人颜色数据 */
#define ROBOT_COMMAND           0x34        /* 机器人位置信息 */
#define CLIENT_MAP_COMMAND      0x35        /* 雷达发送目标位置信息 */
#define BARREL                  0x40        /* 发射机构数据 */
#define MANIFOLD_CTRL           0x50        /* 控制模式 */
#define MODE                    0x60        /* 模式控制 */
#define DEV_ERROR               0xE0        /* 故障信息 */
#define HEARTBEAT               0xF0        /* 心跳数据 */


/**
  * @brief  通讯帧结构体 （BCP通讯协议） 此为最大DATA长度的帧，用于接收中转
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 */
    rt_uint8_t D_ADDR;                 /*! 目标地址 */
    rt_uint8_t ID;                     /*! 功能码 */
    rt_uint8_t LEN;                    /*! 数据长度 */
    rt_int8_t DATA[FRAME_MAX_LEN];     /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPFrameTypeDef;

/**
  * @brief  自瞄发送结构体
  */
typedef  struct
{
    rt_uint16_t head;                   /*! 帧头 */
/*    float pitchAngleGet;          *//*! pitch轴角度 *//*
    float yawAngleGet;              *//*! yaw轴角度 *//*
    rt_uint8_t rotateDirection;        *//*! 旋转方向 1 *//*
    float timeBais;                 *//*! 预测时间偏置 *//*
    float compensateBais;           *//*! 弹道补偿偏置 *//*
    rt_uint8_t gimbal_mode;         *//*! 云台模式 *//*
    rt_uint32_t index;                 *//*! 帧序号 */
    rt_int8_t DATA[FRAME_AUTO_LEN];  /*! 数据内容 FRAME_AUTO_LEN=18 */
    rt_uint8_t index[4];
}__attribute__((packed)) SendFrameTypeDef;

/**
  * @brief  速度方式控制通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                   /*! 帧头 */
    rt_uint8_t D_ADDR;                /*! 目标地址 */
    rt_uint8_t ID;                    /*! 功能码 */
    rt_uint8_t LEN;                   /*! 数据长度 */
    rt_int8_t DATA[FRAME_XYA_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                    /*! 和校验 */
    rt_uint8_t AC;                    /*! 附加校验 */
}__attribute__((packed)) XyaTypeDef;

/**
  * @brief  欧拉角rpy方式控制通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 */
    rt_uint8_t D_ADDR;                 /*! 目标地址 */
    rt_uint8_t ID;                     /*! 功能码 */
    rt_uint8_t LEN;                    /*! 数据长度 */
    rt_int8_t DATA[FRAME_RPY_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) RpyTypeDef;

/**
  * @brief  角/线速度方式控制通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 */
    rt_uint8_t D_ADDR;                 /*! 目标地址 */
    rt_uint8_t ID;                     /*! 功能码 */
    rt_uint8_t LEN;                    /*! 数据长度 */
    rt_int8_t DATA[FRAME_CTRL_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) XyzTypeDef;

/**
  * @brief  里程计方式控制通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 */
    rt_uint8_t D_ADDR;                 /*! 目标地址 */
    rt_uint8_t ID;                     /*! 功能码 */
    rt_uint8_t LEN;                    /*! 数据长度 */
    rt_int8_t DATA[FRAME_ODOM_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) OdomTypeDef;

/**
  * @brief  imu方式控制通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 0XFF */
    rt_uint8_t D_ADDR;                 /*! 目标地址 0X01 */
    rt_uint8_t ID;                     /*! 功能码 0X13 */
    rt_uint8_t LEN;                    /*! 数据长度 40 */
    rt_int8_t DATA[FRAME_IMU_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) ImuTypeDef;

/**
  * @brief  发射机构数据通讯帧结构体
  */
typedef  struct
{
    rt_uint8_t HEAD;                    /*! 帧头 */
    rt_uint8_t D_ADDR;                 /*! 目标地址 */
    rt_uint8_t ID;                     /*! 功能码 */
    rt_uint8_t LEN;                    /*! 数据长度 */
    rt_int8_t DATA[FRAME_SHOOT_LEN];    /*! 数据内容 */
    rt_uint8_t SC;                     /*! 和校验 */
    rt_uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) ShootTypeDef;

/**
  * @brief CDC上下位机通信线程入口函数
  */
void transmission_task_entry(void* argument);

/**
  * @brief 拆分并填充rpy欧拉角数据
  */
void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch,float roll);

/**
  * @brief 和校验，附加校验
  */
void Check_Rpy(RpyTypeDef *frame);

/**
  * @brief 执行发送动作
  */
void Send_to_pc(RpyTypeDef data_r);

/**
  * @brief 执行接收解析动作
  */
void Getdata();

/**
  * @brief 接受回调函数
  */
static rt_err_t usb_input(rt_device_t dev, rt_size_t size);

/**
  * @brief   接收区清空标志位回馈
  */
typedef enum
{
    trans_OK=1,   //执行清空操作
    trans_NO=0,  //不执行清空操作
} trans_back_e;

#endif // RTTHREAD_TRANSMISSION_TASK_H
