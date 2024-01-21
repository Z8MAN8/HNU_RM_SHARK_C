/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*                 ZhengWanshun
*                 Yangshuo
*                 ChenSihan
*/
#ifndef _RC_DBUS_H
#define _RC_DBUS_H

#include <rtthread.h>
#include "rm_config.h"

#define DBUS_RX_BUF_NUM        36u
#define DBUS_FRAME_SIZE        18u     /* DBUS数据帧帧长 */

/**
  * @brief dbus遥控器拨杆值
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};

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
 * @brief 初始化dbus_rc
 *
 * @return rc_dbus_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_dbus_obj_t *dbus_rc_init(void);

/* 解析后的遥控器数据传递给keyboard */
extern rc_dbus_obj_t rc_dbus_obj[2];

#endif /* _RC_DBUS_H */
