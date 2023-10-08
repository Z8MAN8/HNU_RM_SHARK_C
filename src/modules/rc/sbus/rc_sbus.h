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
{
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧上下
    int16_t ch4;   //左侧左右
    int16_t ch5;   //右侧非线性旋钮
    /* 遥控器的拨杆数据，上中下分别为：240、1024、1807 */
    uint8_t sw1;   //右侧长拨杆
    uint8_t sw2;   //左侧长拨杆
    /* 遥控器的二档拨杆数据，上下分别为：240、1807 */
    uint8_t sw3;   //右侧短拨杆
    uint8_t sw4;   //左侧短拨杆
} rc_obj_t;

/**
 * @brief 初始化sbus_rc
 *
 * @return rc_obj_t* 指向NOW和LAST两次数据的数组起始地址
 */
rc_obj_t *sbus_rc_init(void);

#endif /* _RC_SBUS_H */
