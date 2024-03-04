/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include "cmd_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static publisher_t *pub_gim, *pub_chassis, *pub_shoot;
static subscriber_t *sub_gim, *sub_shoot,*sub_trans,*sub_ins;
static struct gimbal_cmd_msg gim_cmd;
static struct gimbal_fdb_msg gim_fdb;
static struct shoot_cmd_msg  shoot_cmd;
static struct shoot_fdb_msg  shoot_fdb;
static struct chassis_cmd_msg chassis_cmd;
static struct trans_fdb_msg  trans_fdb;
static struct ins_msg ins_data;

static rc_dbus_obj_t *rc_now, *rc_last;

static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/*发射停止标志位*/
static int trigger_flag=0;
/*堵转电流反转记次*/
static int reverse_cnt;
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;

static int pitch_cnt=0;
/*判断上位机角度是否更新数值*/
static gim_auto_judge yaw_auto;
static gim_auto_judge pitch_auto;

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd_dbus(void);
static void remote_to_cmd_sbus(void);
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式
/*储存鼠标坐标数据*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;
float Ballistic;  //对自瞄数据进行手动鼠标弹道补偿
/* --------------------------------- cmd线程入口 -------------------------------- */
static float cmd_dt;

void cmd_thread_entry(void *argument)
{
    static float cmd_start;

    cmd_pub_init();
    cmd_sub_init();

    rc_now = dbus_rc_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    /* 初始化拨杆为上位 */
    rc_now->sw1 = RC_UP;
    rc_now->sw2 = RC_UP;
    //rc_now->sw3 = RC_UP;
    //rc_now->sw4 = RC_UP;

    LOG_I("Cmd Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        cmd_sub_pull();

        /* 将遥控器原始数据转换为控制指令 */

        #ifdef BSP_USING_RC_KEYBOARD
        PC_Handle_kb();//处理PC端键鼠控制
        #endif
        /* 将遥控器原始数据转换为控制指令 */
        #ifdef BSP_USING_RC_DBUS
        remote_to_cmd_dbus();
        #endif
        /* 将遥控器原始数据转换为控制指令 */
        #ifdef BSP_USING_RC_SBUS
        remote_to_cmd_sbus();
        #endif

        /* 更新发布该线程的msg */
        cmd_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
                LOG_E("Cmd Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者初始化
 */
static void cmd_pub_init(void)
{
    pub_gim = pub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot= pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_cmd);
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
    sub_shoot= sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(sub_gim, &gim_fdb);
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_trans,&trans_fdb);
    sub_get_msg(sub_ins, &ins_data);
}

/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef BSP_USING_RC_DBUS
struct dbus_mode_flag_t
{
    uint8_t chassis_flag;
    uint8_t shoot_flag;
};
static int cnt_patrol;
static struct dbus_mode_flag_t dbus_mode_flag;
static void remote_to_cmd_dbus(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx = rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy = rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw = rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW /*-fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW*/;
        gim_cmd.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT /*- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT*/;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =ins_data.pitch;

    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO) {

        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;//上位机自瞄
        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT /*- Ballistic * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT*/;//上位机自瞄

    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;

    }
    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    // 左拨杆sw2为上时，底盘和云台均RELAX；为中时，云台为GYRO；为下时，云台为AUTO。
    // 右拨杆sw1为上时，底盘为FOLLOW；为中时，底盘为OPEN；为下时，底盘为SPIN。





   /* switch (rc_now->sw1)
    {
    case RC_UP:
        if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX)
        {
            chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
        }
        else
        {
            //TODO:把开环模式改成释放，让云台归中到底盘
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        break;

    case RC_DN:
        if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX)
        {
            chassis_cmd.ctrl_mode = CHASSIS_SPIN;
            // TODO：考虑将陀螺转速改为变量，可以手动或自动调整转速
            if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
            {
                chassis_cmd.vw = (float) (rc_now->ch5) / 784.0 * 5.0; // 小陀螺转速
            }
            if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
            {
                chassis_cmd.vw=(float)(rc_now->ch5) / 784.0 * 5.0; // 小陀螺转速
                chassis_cmd.vx=2000*(float)(rc_now->ch5) / 784.0;
                chassis_cmd.vy=2000*(float)(rc_now->ch5) / 784.0;
            }

        }
        else
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        break;
    }*/

    /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
    /*右拨杆三种模式：停止 手动 自动*/
    switch(rc_now->sw2)
    {
    case RC_UP:
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        /*放开状态下，gim不接收值*/
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        break;
    case RC_MI:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode=CHASSIS_FOLLOW_GIMBAL;
            }
            else chassis_cmd.ctrl_mode=CHASSIS_OPEN_LOOP;
        }

        break;
    case RC_DN:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode=CHASSIS_FOLLOW_GIMBAL;
            }
        }
           /* chassis_cmd.ctrl_mode=CHASSIS_OPEN_LOOP;
            chassis_cmd.vx=trans_fdb.line_x*1000;
            chassis_cmd.vy=trans_fdb.line_y*1000;
            chassis_cmd.vw=-trans_fdb.angle_z;*/
        break;
    }

//    /*左拨杆计数器*/
//    /*摩擦轮开关*/
//    if (rc_now->sw1==RC_UP&&rc_now->wheel<-10&&dbus_mode_flag.shoot_flag==0)
//    {
//        dbus_mode_flag.shoot_flag++;
//        shoot_cmd.friction_status=1;
//    }
//    else if(rc_now->sw1==RC_DN&&rc_now->wheel<-10&&dbus_mode_flag.chassis_flag==0)/*跟随模式和小陀螺模式切换*/
//    {
//        dbus_mode_flag.chassis_flag++;
//        chassis_cmd.ctrl_mode=CHASSIS_SPIN;
//        chassis_cmd.vw=3;
//    }
//    if (rc_now->sw1==RC_MI&&rc_now->wheel<-200)
//    {
//        dbus_mode_flag.shoot_flag=0;
//        dbus_mode_flag.chassis_flag=0;
//        chassis_cmd.ctrl_mode=CHASSIS_FOLLOW_GIMBAL;
//        shoot_cmd.friction_status=0;
//    }
//
//    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
//    {
//        chassis_cmd.vw=3;
//    }

    /*左拨杆计数器*/
    /*摩擦轮开关*/
    if (rc_now->sw1==RC_UP)
    {
        shoot_cmd.friction_status=0;
    }
    else if(rc_now->sw1==RC_MI)/*跟随模式和小陀螺模式切换*/
    {
        shoot_cmd.friction_status=1;
    }
    if (rc_now->sw1==RC_DN)
    {
        chassis_cmd.ctrl_mode=CHASSIS_SPIN;
        shoot_cmd.friction_status=1;
    }

    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd.vw=3;
    }
    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*单发/三联模式发射判断*/
    if (rc_now->wheel>=-640)
        trigger_flag = 1;
    if (rc_now->wheel<=-640 && trigger_flag)
        {//判断是否要开火
                    shoot_cmd.trigger_status = TRIGGER_ON;
                    trigger_flag = 0;
                    shoot_cmd.ctrl_mode=SHOOT_ONE;
        }
    else
    {
        shoot_cmd.trigger_status = TRIGGER_OFF;
    }
     /*连发模式*/
    if(rc_now->wheel>=50)
    {
        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
         shoot_cmd.shoot_freq=(int16_t)(50+rc_now->wheel*5);
    }
    /*堵弹反转检测*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
    /*舵机开盖关盖*/
    if(rc_now->sw1==RC_UP&&rc_now->wheel<=-640)
    {
         shoot_cmd.cover_open=1;
    }
    else
{
         shoot_cmd.cover_open=0;
    }
}
#endif

/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
/**
 * @brief 将遥控器数据转换为控制指令
 */
#ifdef BSP_USING_RC_SBUS
static void remote_to_cmd_sbus(void)
{
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    *rc_last = *rc_now;
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx += rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy += rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw += rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;
    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW -fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT - fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =ins_data.pitch;

    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO) {

        gim_cmd.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;//上位机自瞄
        gim_cmd.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT - Ballistic * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;//上位机自瞄

    }
    /* 限制云台角度 */

    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);




    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/
    // 左拨杆sw2为上时，底盘和云台均RELAX；为中时，云台为GYRO；为下时，云台为AUTO。
    // 右拨杆sw1为上时，底盘为FOLLOW；为中时，底盘为OPEN；为下时，底盘为SPIN。
    /*if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;

    }
    switch (rc_now->sw2)
    {
    case RC_UP:
        if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX)
        {
            chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
        }
        else
        {
            //TODO:把开环模式改成释放，让云台归中到底盘
            //chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        break;
*//*    case RC_MI:
        chassis_cmd.ctrl_mode = CHASSIS_OPEN_LOOP;
        break;*//*
    case RC_DN:
        if(gim_cmd.ctrl_mode != GIMBAL_INIT && gim_cmd.ctrl_mode != GIMBAL_RELAX)
        {
            chassis_cmd.ctrl_mode = CHASSIS_SPIN;
            // TODO：考虑将陀螺转速改为变量，可以手动或自动调整转速
            if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
            {
                chassis_cmd.vw = (float) (rc_now->ch5) / 784.0 * 5.0; // 小陀螺转速
            }
            if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
            {
                chassis_cmd.vw=(float)(rc_now->ch5) / 784.0 * 5.0; // 小陀螺转速
                chassis_cmd.vx=2000*(float)(rc_now->ch5) / 784.0;
                chassis_cmd.vy=2000*(float)(rc_now->ch5) / 784.0;
            }

        }
        else
        {
            chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        }
        break;
    }
    *//* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 *//*

    switch(rc_now->sw3)
    {
    case RC_UP:
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        *//*放开状态下，gim不接收值*//*
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        break;
    case RC_MI:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {*//* 判断上次状态是否为RELAX，是则先归中 *//*
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
            }
        }
        break;
    case RC_DN:
        if(gim_cmd.last_mode == GIMBAL_RELAX)
        {*//* 判断上次状态是否为RELAX，是则先归中 *//*
            gim_cmd.ctrl_mode = GIMBAL_INIT;
        }
        else
        {
            if(gim_fdb.back_mode == BACK_IS_OK)
            {*//* 判断归中是否完成 *//*
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
            }
        }
        break;
    }
*/
    /*--------------------------------------------------发射模块状态机--------------------------------------------------------------*/

    /*if(rc_now->sw3!=RC_UP&&gim_cmd.ctrl_mode!=GIMBAL_AUTO)//判断总开关是否停止发射
    {
        switch (rc_now->sw1)
        {
            *//*判断是否处于可发射状态*//*
            //TODO:由于遥控器拨杆档位限制,目前连发模式还未写进状态机。两档拨杆具体值由遥控器确定，现在待定。
            case RC_DN:
                if (rc_now->ch6 <= 775)
                    trigger_flag = 1;
                if (rc_now->ch6 >= 775 && trigger_flag)
                {//判断是否要开火
                    shoot_cmd.trigger_status = TRIGGER_ON;
                    trigger_flag = 0;
                }
                else shoot_cmd.trigger_status = TRIGGER_OFF;
                *//*判断发射模式是三连发还是全自动*//*
                switch (rc_now->sw4)
                {
                    case RC_UP:
                        shoot_cmd.ctrl_mode = SHOOT_ONE;
                        break;
                    case RC_DN:
                        shoot_cmd.ctrl_mode = SHOOT_COUNTINUE;
                        break;
                }
                break;
            case RC_UP:
                shoot_cmd.ctrl_mode = SHOOT_STOP;
                shoot_cmd.trigger_status = TRIGGER_OFF;
                break;
        }
    }
    else if (gim_cmd.ctrl_mode==GIMBAL_AUTO&&rc_now->sw4==RC_DN)
    {
        shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
    }
    else
    {
        shoot_cmd.ctrl_mode==SHOOT_STOP;
    }
    *//*堵弹反转检测*//*
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }

    // TODO: 添加弹频和弹速控制
    if (rc_now->ch6>0)
    {
        shoot_cmd.shoot_freq = rc_now->ch6 / RC_MAX_VALUE*10;//连发模式拨弹电机转速
    }
    else if(rc_now->ch6<=-775)
    {
         shoot_cmd.cover_open=1;
    }
    else
    {
         shoot_cmd.cover_open=0;
    }*/
}
#endif
