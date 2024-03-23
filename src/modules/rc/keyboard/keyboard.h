#ifndef RTTHREAD_KEYBOARD_H
#define RTTHREAD_KEYBOARD_H

#include <rtthread.h>
#include "rm_config.h"

/**
  * @brief     底盘运动速度快慢模式
  */
typedef enum
{
    NORMAL_MODE = 0,    //正常模式
    FAST_MODE,          //快速模式
    SLOW_MODE,          //慢速模式
} kb_move_e;

/**
  * @brief     鼠标按键状态类型枚举
  */
typedef enum
{
    KEY_RELEASE = 0,    //没有按键按下
    KEY_WAIT_EFFECTIVE, //等待按键按下有效，防抖
    KEY_PRESS_ONCE,     //按键按下一次的状态
    KEY_PRESS_DOWN,     //按键已经被按下
    KEY_PRESS_LONG,     //按键长按状态
} kb_state_e;

/**
  * @brief     键盘鼠标数据结构体
  */
typedef struct
{
    /* 键盘模式使能标志 */
    uint8_t kb_enable;

    /* 鼠标键盘控制模式下的底盘移动速度目标值 */
    float vx;          //底盘前进后退目标速度
    float vy;          //底盘左右平移目标速度
    float vw;          //底盘旋转速度
    float max_spd;     //运动最大速度

    /* 左右按键状态 */
    kb_state_e lk_sta; //左侧按键状态
    kb_state_e rk_sta; //右侧按键状态

    /* 键盘按键状态 */
    kb_state_e e_sta; //E键按键状态
    kb_state_e f_sta; //E键按键状态

    uint16_t lk_cnt;
    uint16_t rk_cnt;

    /* 运动模式，键盘控制底盘运动快慢 */
    kb_move_e move_mode;

} km_control_t;

extern km_control_t km;

/**
  * @brief     PC 处理键盘鼠标数据函数
  */
void PC_Handle_kb(void);

#endif //RTTHREAD_KEYBOARD_H
