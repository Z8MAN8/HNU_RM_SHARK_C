/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

// TODO: 移植底盘运动逆解算

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static publisher_t *pub_chassis;
static subscriber_t *sub_cmd;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */
static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
static pid_config_t chassis_follow_config = INIT_PID_CONFIG(CHASSIS_KP_V_FOLLOW, CHASSIS_KI_V_FOLLOW, CHASSIS_KD_V_FOLLOW, CHASSIS_INTEGRAL_V_FOLLOW, CHASSIS_MAX_V_FOLLOW,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];  // 底盘电机实例
static int16_t motor_ref[4]; // 电机控制期望值

static void chassis_motor_init();

/* --------------------------------- 底盘运动学解算 --------------------------------- */
/* 根据宏定义选择的底盘类型使能对应的解算函数 */
#ifdef BSP_CHASSIS_OMNI_MODE
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = omni_calc;
#endif /* BSP_CHASSIS_OMNI_MODE */
#ifdef BSP_CHASSIS_MECANUM_MODE
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
#endif /* BSP_CHASSIS_MECANUM_MODE */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float cmd_dt;

void chassis_thread_entry(void *argument)
{
    static float cmd_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();

    LOG_I("Chassis Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        chassis_sub_pull();

        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }

        switch (chassis_cmd.ctrl_mode)
        {
        case CHASSIS_RELAX:
            for (uint8_t i = 0; i < 4; i++)
            {
                dji_motor_relax(chassis_motor[i]);
            }
            break;
        case CHASSIS_FOLLOW_GIMBAL:
            chassis_cmd.vw = pid_calculate(follow_pid, chassis_cmd.offset_angle, 0);
            /* 底盘运动学解算 */
            absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_SPIN:
            absolute_cal(&chassis_cmd, chassis_cmd.offset_angle);
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_OPEN_LOOP:
            chassis_calc_moto_speed(&chassis_cmd, motor_ref);
            break;
        case CHASSIS_STOP:
            rt_memset(motor_ref, 0, sizeof(motor_ref));
            break;
        case CHASSIS_FLY:
            break;
        case CHASSIS_AUTO:
            break;
        default:
            for (uint8_t i = 0; i < 4; i++)
            {
                dji_motor_relax(chassis_motor[i]);
            }
            break;
        }

        /* 更新发布该线程的msg */
        chassis_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
            LOG_E("Chassis Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{

}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{

}

/* --------------------------------- 电机控制相关 --------------------------------- */
/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
}

static rt_int16_t motor_control_0(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_ref[0]);
    return set;
}

static rt_int16_t motor_control_1(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_ref[1]);
    return set;
}

static rt_int16_t motor_control_2(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_ref[2]);
    return set;
}

static rt_int16_t motor_control_3(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_ref[3]);
    return set;
}

/* 底盘每个电机对应的控制函数 */
static void *motor_control[4] =
        {
                motor_control_0,
                motor_control_1,
                motor_control_2,
                motor_control_3,
        };

// TODO：将参数都放到配置文件中，通过宏定义进行替换
motor_config_t chassis_motor_config[4] =
        {
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x201,
                        .controller = &chassis_controller[0],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x202,
                        .controller = &chassis_controller[1],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x203,
                        .controller = &chassis_controller[2],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS,
                        .rx_id = 0x204,
                        .controller = &chassis_controller[3],
                }
        };

/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR, CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);
    }

    follow_pid = pid_register(&chassis_follow_config);
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#ifdef BSP_CHASSIS_OMNI_MODE
/**
 * @brief 全向轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio; //left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio; //forward
    wheel_rpm[2] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio; //right
    wheel_rpm[3] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio; //back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t)); //copy the rpm to out_speed
}
#endif /* BSP_CHASSIS_OMNI_MODE */

#ifdef BSP_CHASSIS_MECANUM_MODE
/**
 * @brief 麦克纳姆轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    static float rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx - cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( cmd->vx + cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-cmd->vx + cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-cmd->vx - cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

/**
  * @brief  将云台坐标转换为底盘坐标
  * @param  cmd 底盘指令值，使用其中的速度
  * @param  angle 云台相对于底盘的角度
  */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle)
{
    float angle_hd = -(angle/RADIAN_COEF);
    float vx = cmd->vx;
    float vy = cmd->vy;

    //保证底盘是相对摄像头做移动
    cmd->vx = vx * cos(angle_hd) + vy * sin(angle_hd);
    cmd->vy = -vx * sin(angle_hd) + vy * cos(angle_hd);
}
