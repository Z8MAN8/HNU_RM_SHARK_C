/*
* Change Logs:
* Date            Author          Notes
* 2023-10-09      ChenSihan     1.0.0version 发射线程模块
* 2023-10-14      ChenSihan     发射逻辑优化 pid优化
*/
#include "shoot_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

//TODO: 弹频和弹速的控制应在cmd线程中决策

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;

static publisher_t *pub_shoot;
static subscriber_t *sub_cmd;

static void shoot_pub_init(void);
static void shoot_sub_init(void);
static void shoot_pub_push(void);
static void shoot_sub_pull(void);
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */

/*舵机pwm设备*/
static struct rt_device_pwm *servo_cover_dev;  // 弹仓盖舵机

/*发射模块电机使用数量*/
#define SHT_MOTOR_NUM 3

/*发射模块电机编号：分别为左摩擦轮电机 右摩擦轮电机 拨弹电机*/
#define RIGHT_FRICTION 0
#define LEFT_FRICTION 1
#define TRIGGER_MOTOR 2

/*pid环数结构体*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
}sht_controller[SHT_MOTOR_NUM];

/*电机注册初始化数据*/
motor_config_t shoot_motor_config[SHT_MOTOR_NUM] ={
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = RIGHT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[RIGHT_FRICTION],
    },
    {
        .motor_type = M3508,
        .can_name = CAN_GIMBAL,
        .rx_id = LEFT_FRICTION_MOTOR_ID,
        .controller = &sht_controller[LEFT_FRICTION],
    },
    {
        .motor_type = M2006,
        .can_name = CAN_GIMBAL,
        .rx_id = TRIGGER_MOTOR_ID,
        .controller = &sht_controller[TRIGGER_MOTOR],
    }
};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // 发射器电机实例
static float shoot_motor_ref[SHT_MOTOR_NUM]; // 电机控制期望值

/*函数声明*/
static void shoot_motor_init();
static void servo_init();
static rt_int16_t motor_control_right(dji_motor_measure_t measure);
static rt_int16_t motor_control_left(dji_motor_measure_t measure);
static rt_int16_t motor_control_trigger(dji_motor_measure_t measure);

/**
 * @brief shoot线程入口函数
 */
void shoot_task_entry(void* argument)
{
    static float sht_dt;
    static float sht_start;
    static int total_angle_flag=0;//转子角度标志位，防止切换设计模式时拨弹电机反转
    static int servo_cvt_num;

    shoot_motor_init();
    shoot_pub_init();
    shoot_sub_init();
    servo_init();
/*----------------------射击状态初始化----------------------------------*/
    shoot_cmd.ctrl_mode=SHOOT_STOP;
    shoot_cmd.trigger_status=TRIGGER_OFF;
    shoot_motor_ref[TRIGGER_MOTOR]=0;
    LOG_I("Shoot Task Start");
    for (;;)
    {
        sht_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        shoot_sub_pull();

        /* 弹仓盖舵机控制 */
        if (shoot_cmd.cover_open == 1)
            rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 2000000);
        else
            rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 780000);

        /* 电机控制启动 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
        {
            dji_motor_enable(sht_motor[i]);
        }

        /*控制模式判断*/
        switch (shoot_cmd.ctrl_mode)
        {
        case SHOOT_STOP:
            shoot_motor_ref[TRIGGER_MOTOR] = 0;
            shoot_motor_ref[RIGHT_FRICTION] =0;
            shoot_motor_ref[LEFT_FRICTION] = 0;
            total_angle_flag=0;
            break;

        case SHOOT_ONE:
            shoot_motor_ref[RIGHT_FRICTION] = 3000;//摩擦轮常转
            shoot_motor_ref[LEFT_FRICTION] = -3000;
            /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
            if(total_angle_flag == 0)
            {
                shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                total_angle_flag=1;
            }
            if (shoot_cmd.trigger_status == TRIGGER_ON)
            {
                shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_45_TO_ANGLE * 36;//M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍
                shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
            }
            shoot_fdb.shoot_mode=SHOOT_OK;
            break;

        case SHOOT_THREE:
            shoot_motor_ref[RIGHT_FRICTION] = 3000;//摩擦轮常转
            shoot_motor_ref[LEFT_FRICTION] = -3000;
            /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
            if(total_angle_flag == 0)
            {
                shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                total_angle_flag = 1;
            }
            if (shoot_cmd.trigger_status == TRIGGER_ON)
            {
                shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + 3 * TRIGGER_MOTOR_45_TO_ANGLE * 36;//M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍
                shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
            }
            shoot_fdb.shoot_mode=SHOOT_OK;
            break;

        case SHOOT_COUNTINUE:
            shoot_motor_ref[RIGHT_FRICTION] = 3000;//摩擦轮常转
            shoot_motor_ref[LEFT_FRICTION] = -3000;
            shoot_motor_ref[TRIGGER_MOTOR] = shoot_cmd.shoot_freq;//自动模式的时候，只用速度环控制拨弹电机
            if (shoot_cmd.shoot_freq>=1500&&shoot_cmd.shoot_freq<=2500)
            {
                shoot_motor_ref[TRIGGER_MOTOR] = 2000;//自动模式的时候，只用速度环控制拨弹电机
            }
            else if(shoot_cmd.shoot_freq>2500)
            {
                shoot_motor_ref[TRIGGER_MOTOR] = 4000;
            }
             else{
                shoot_motor_ref[TRIGGER_MOTOR] = 0;
             }
            total_angle_flag = 0;
            shoot_fdb.shoot_mode = SHOOT_OK;
            break;

        default:
            for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
            {
                dji_motor_relax(sht_motor[i]); // 错误情况电机全部松电
            }
            shoot_fdb.shoot_mode=SHOOT_ERR;
            break;
        }
        /* 更新发布该线程的msg */
        shoot_pub_push();

        //TODO:单独调试shoot模块时打开，用于更新上个模式
        shoot_cmd.last_mode=shoot_cmd.ctrl_mode;

        /* 用于调试监测线程调度使用 */
        sht_dt = dwt_get_time_ms() - sht_start;
        if (sht_dt > 1)
            LOG_E("Shoot Task is being DELAY! dt = [%f]", &sht_dt);
        rt_thread_mdelay(1);
    }
}

/**
 * @brief 舵机初始化
 */
static void servo_init(){
    servo_cover_dev=(struct rt_device_pwm *) rt_device_find(PWM_COVER);
    if(servo_cover_dev == RT_NULL)
    {
        LOG_E("Can't find cover servo pwm device!");
        return;
    }
    rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 780000);
    rt_pwm_enable(servo_cover_dev, PWM_COVER_CH);
}

/**
 * @brief shoot 线程电机初始化
 */
static void shoot_motor_init(){
    /* -------------------------------------- right_friction 右摩擦轮电机 ----------------------------------------- */
    pid_config_t right_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V,RIGHT_INTEGRAL_V,RIGHT_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[RIGHT_FRICTION].pid_speed = pid_register(& right_speed_config);

/* ------------------------------------------- left_friction 左摩擦轮电机------------------------------------------------- */
    pid_config_t left_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[LEFT_FRICTION].pid_speed = pid_register(&left_speed_config);

/* ------------------------------------------------  拨弹电机------------------------------------------------------------------------- */
    pid_config_t toggle_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t toggle_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[TRIGGER_MOTOR].pid_speed = pid_register(&toggle_speed_config);
    sht_controller[TRIGGER_MOTOR].pid_angle = pid_register(&toggle_angle_config);

/* ---------------------------------- shoot电机初注册---------------------------------------------------------------------------------------- */
    sht_motor[TRIGGER_MOTOR] = dji_motor_register(&shoot_motor_config[TRIGGER_MOTOR], motor_control_trigger);
    sht_motor[LEFT_FRICTION] = dji_motor_register(&shoot_motor_config[LEFT_FRICTION], motor_control_left);
    sht_motor[RIGHT_FRICTION] = dji_motor_register(&shoot_motor_config[RIGHT_FRICTION], motor_control_right);
}

/*右摩擦轮电机控制算法*/
static rt_int16_t motor_control_right(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[RIGHT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[RIGHT_FRICTION]);
    return set;
}

/*右摩擦轮电机控制算法*/
static rt_int16_t motor_control_left(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[LEFT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[LEFT_FRICTION]);
    return set;
}

/*拨弹电机控制算法*/
static rt_int16_t motor_control_trigger(dji_motor_measure_t measure){
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static rt_int16_t send_data;        // 最终发送给电调的数据

    /*拨弹电机采用串级pid，一个角度环和一个速度环*/
    pid_speed = sht_controller[TRIGGER_MOTOR].pid_speed;
    pid_angle = sht_controller[TRIGGER_MOTOR].pid_angle;
    get_angle=measure.total_angle;
    get_speed=measure.speed_rpm;

    /* 切换模式需要清空控制器历史状态 */
    if(shoot_cmd.ctrl_mode != shoot_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    /*pid计算输出*/
    if (shoot_cmd.ctrl_mode==SHOOT_ONE||shoot_cmd.ctrl_mode==SHOOT_THREE) //非连发模式的时候，用双环pid控制拨弹电机
    {
        pid_out_angle = (int16_t) pid_calculate(pid_angle, get_angle, shoot_motor_ref[TRIGGER_MOTOR]);  // 编码器增长方向与imu相反
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
    }
    /*pid计算输出*/
    else if(shoot_cmd.ctrl_mode==SHOOT_COUNTINUE||shoot_cmd.ctrl_mode==SHOOT_STOP)//自动模式的时候，只用速度环控制拨弹电机
    {
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, shoot_motor_ref[TRIGGER_MOTOR] );
    }
    return send_data;
}

/**
 * @brief shoot 线程中所有发布者初始化
 */
static void shoot_pub_init(){
    pub_shoot = pub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
}
/**
 * @brief shoot 线程中所有订阅者初始化
 */
static void shoot_sub_init(){
    sub_cmd = sub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}
/**
 * @brief shoot 线程中所有发布者推送更新话题
 */
static void shoot_pub_push(){
    pub_push_msg(pub_shoot, &shoot_fdb);
}
/**
 * @brief shoot 线程中所有订阅者推送更新话题
 */
static void shoot_sub_pull(){
    sub_get_msg(sub_cmd, &shoot_cmd);
}
