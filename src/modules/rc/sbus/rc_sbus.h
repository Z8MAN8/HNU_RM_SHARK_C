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

#include "rt-thread/include/rtthread.h"

#define SBUS_RX_BUF_NUM 36u
#define SBUS_FRAME_SIZE 25u

/**
  * @brief 遥控器拨杆值
  */
enum {
    RC_UP = 240,
    RC_MI = 0,
    RC_DN = 15,
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
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_obj_t *sbus_rc_init(void);

#endif /* _RC_SBUS_H */
