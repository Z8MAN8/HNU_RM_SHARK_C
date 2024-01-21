#include "keyboard.h"
#include "rm_module.h"
#include "rm_algorithm.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  800   //ms
/* key acceleration time */
#define KEY_ACC_TIME     1000  //ms

km_control_t km;

int16_t delta_spd = MAX_CHASSIS_VX_SPEED*1.0f/KEY_ACC_TIME*GIMBAL_PERIOD;

/**
  * @brief     鼠标按键状态机
  * @param[in] sta: 按键状态指针
  * @param[in] key: 按键键值
  */
static void key_fsm(kb_state_e *sta, uint8_t key)
{
    switch (*sta)
    {
        case KEY_RELEASE:
        {
            if (key)
                *sta = KEY_WAIT_EFFECTIVE;
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key)
                *sta = KEY_PRESS_ONCE;
            else
                *sta = KEY_RELEASE;
        }break;


        case KEY_PRESS_ONCE:
        {
            if (key)
            {
                *sta = KEY_PRESS_DOWN;
                if (sta == &km.lk_sta)
                    km.lk_cnt = 0;
                else
                    km.rk_cnt = 0;
            }
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_PRESS_DOWN:
        {
            if (key)
            {
                if (sta == &km.lk_sta)
                {
                    if (km.lk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        *sta = KEY_PRESS_LONG;
                }
                else
                {
                    if (km.rk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        *sta = KEY_PRESS_LONG;
                }
            }
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_PRESS_LONG:
        {
            if (!key)
            {
                *sta = KEY_RELEASE;
            }
        }break;

        default:
            break;

    }
}

/**
  * @brief     PC 处理键盘鼠标数据函数
  */
void PC_Handle_kb(void)
{
    if (rc_dbus_obj[0].kb.bit.SHIFT)
    {
        km.move_mode = FAST_MODE;
        km.max_spd = 3500;
    }
    else if (rc_dbus_obj[0].kb.bit.CTRL)
    {
        km.move_mode = SLOW_MODE;
        km.max_spd = 2500;
    }
    else
    {
        km.move_mode = NORMAL_MODE;
        km.max_spd = 3000;
    }

    //add ramp
    if (rc_dbus_obj[0].kb.bit.W)
        km.vy += delta_spd;
    else if (rc_dbus_obj[0].kb.bit.S)
        km.vy -= delta_spd;
    else
        km.vy = 0;

    if (rc_dbus_obj[0].kb.bit.A)
        km.vx += -delta_spd;
    else if (rc_dbus_obj[0].kb.bit.D)
        km.vx += delta_spd;
    else
        km.vx = 0;

    VAL_LIMIT(km.vx, -km.max_spd, km.max_spd);
    VAL_LIMIT(km.vy, -km.max_spd, km.max_spd);

    VAL_LIMIT(km.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(km.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);

    key_fsm(&km.lk_sta, rc_dbus_obj[0].mouse.l);
    key_fsm(&km.rk_sta, rc_dbus_obj[0].mouse.r);
}
