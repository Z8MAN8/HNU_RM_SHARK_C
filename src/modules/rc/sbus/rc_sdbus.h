/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*                 ZhengWanshun
*                 Yangshuo
*                 ChenSihan
*/
#ifndef _RC_SBUS_H
#define _RC_SBUS_H

#include <rtthread.h>
#include "rm_config.h"

/**
  * @brief sbus遥控器拨杆值
  */
enum {
    RC_UP = 240,
    RC_MI = 0,
    RC_DN = 15,
};

/**
  * @brief dbus遥控器拨杆值
  */
enum
{
    RC_DBUS_UP = 1,
    RC_DBUS_MI = 3,
    RC_DBUS_DN = 2,
};

typedef struct
{   /* 摇杆最终值为：-784~783 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧上下
    int16_t ch4;   //左侧左右
    /* FS-i6x旋钮为线性，左右最终值为：-784~783 */
    int16_t ch5;   //左侧线性旋钮
    int16_t ch6;   //右侧线性旋钮
    /* 遥控器的拨杆数据，上(中)下最终值分别为：240、（0）、15 */
    uint8_t sw1;   //SWA，二档
    uint8_t sw2;   //SWB，二档
    uint8_t sw3;   //SWC，三档
    uint8_t sw4;   //SWD，二档
} rc_obj_t;

/**
  * @brief     解析后的遥控器数据结构体
  */
typedef struct
{
    /* 遥控器的通道数据，数值范围：-660 ~ 660 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧左右
    int16_t ch4;   //左侧上下

    /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
    uint8_t sw1;   //左侧拨杆
    uint8_t sw2;   //右侧拨杆

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        }bit;
    }kb;

    /* 遥控器左侧拨轮数据数值范围:（左）660 ~ -660(右) */
    int16_t wheel;
}rc_dbus_obj_t;

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_obj_t *sbus_rc_init(void);

/**
 * @brief 初始化dbus_rc
 *
 * @return rc_dbus_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_dbus_obj_t *dbus_rc_init(void);

#endif /* _RC_SBUS_H */
