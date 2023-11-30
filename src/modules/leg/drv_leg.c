/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#include "drv_leg.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "drv.leg"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <stdio.h>

#define LEFT 0
#define RIGHT 1
static uint8_t idx = 0; // register idx,是该文件的全局轮腿索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static leg_obj_t leg_obj[2];

static void _leg_resolve_pos(leg_obj_t *leg)
{
    float l1 = leg->l1;
    float l2 = leg->l2;
    /* xdb = xd-xb; ydb = yd-yb; */
    float xdb = leg->moto_distance + leg->l1 * (arm_cos_f32(leg->phi4) - arm_cos_f32(leg->phi1));
    float ydb = leg->l1 * (arm_sin_f32(leg->phi4) - arm_sin_f32(leg->phi1));

    float A0 = 2 * l2 * xdb;
    float B0 = 2 * l2 * ydb;
    float C0 = xdb*xdb + ydb*ydb;
    float phi2 = 2.0f * atan2f(B0 + sqrtf(A0*A0 + B0*B0 - C0*C0), (A0+C0));

    leg->U2 = phi2;

    /*计算B坐标*/
    leg->CoorB[0] = l1 * arm_cos_f32(leg->phi1)/* - leg->moto_distance_half*/;
    leg->CoorB[1] = l1 * arm_sin_f32(leg->phi1);
    /*计算C坐标*/
    leg->CoorC[0] = leg->CoorB[0] + l2 * arm_cos_f32(leg->U2);
    leg->CoorC[1] = leg->CoorB[1] + l2 * arm_sin_f32(leg->U2);
    /*计算D坐标*/
    leg->CoorD[0] = l1 * arm_cos_f32(leg->phi4) + leg->moto_distance;
    leg->CoorD[1] = l1 * arm_sin_f32(leg->phi4);
    /*计算u3*/
    leg->U3 = 2.0f * atan2f(leg->CoorD[1]-leg->CoorC[1], leg->CoorD[0]-leg->CoorC[0]);

    /*输出摆长摆角*/
    leg->PendulumRadian = atan2f(leg->CoorC[1],leg->CoorC[0] - leg->moto_distance_half);
    leg->PendulumLength = sqrtf((leg->CoorC[0] - leg->moto_distance_half)*(leg->CoorC[0] - leg->moto_distance_half) + leg->CoorC[1]*leg->CoorC[1]);
}

static int8_t _input_leg_angle(leg_obj_t *leg, float phi4, float phi1)
{
    leg->phi4 = phi4;
    leg->phi1 = phi1;

    if(phi4>leg->phi4_max)
    {leg->leg_state = LEG_ERROR;}
    else if(phi1<leg->phi1_min)
    {leg->leg_state = LEG_ERROR;}
    else
    {leg->leg_state = LEG_NORMAL;}

    return (uint8_t)leg->leg_state;
}

/*Leg motors*/
/*FT = [PendulumForce PendulumTorque]   Torque = [Motor3Torque(backmotor)  Motor2Torque(frontmotor)] */
static void _VMC_calculation(leg_obj_t *leg, float *FT, float *Tmotor)
{
    /*计算VMC*/
    volatile float q00,q01,q10,q11;
    /*中间变量*/
    volatile float sin32 = arm_sin_f32(leg->U3-leg->U2);
    volatile float sin12 = arm_sin_f32(leg->phi1 - leg->U2);
    volatile float sin34 = arm_sin_f32(leg->U3 - leg->phi4);


    q00 = leg->l1 * arm_sin_f32(leg->PendulumRadian - leg->U3) * sin12 / sin32;
    q01 = leg->l1 * arm_cos_f32(leg->PendulumRadian - leg->U3) * sin12 / (leg->PendulumLength *sin32);
    q10 = leg->l1 * arm_sin_f32(leg->PendulumRadian - leg->U2) * sin34 / sin32;
    q11 = leg->l1 * arm_cos_f32(leg->PendulumRadian - leg->U2) * sin34 / (leg->PendulumLength *sin32);

    /*矩阵乘法*/
    Tmotor[0] = q00*FT[0] + q01*FT[1];
    Tmotor[1] = q10*FT[0] + q11*FT[1];
}

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t *leg_register(leg_config_t *config/* , void *control */)
{
    leg_obj[idx].l1 = config->l1;
    leg_obj[idx].l2 = config->l2;
    leg_obj[idx].moto_distance = config->moto_distance;
    leg_obj[idx].moto_distance_half = config->moto_distance / 2;
    leg_obj[idx].phi4_max = PI/2;
    leg_obj[idx].phi1_min = PI/2;
    leg_obj[idx].leg_state = LEG_ERROR;
    leg_obj[idx].PendulumRadian = PI/2;
    leg_obj[idx].resolve = _leg_resolve_pos;
    leg_obj[idx].VMC_cal = _VMC_calculation;
    leg_obj[idx].input_leg_angle = _input_leg_angle;

    return &leg_obj[idx++];
}

// 测试接口
/* void input(int argc,char** argv)  //(float phi4, float phi1) // 直接输入角度
{
    float phi1, phi4;
    sscanf(argv[1],"%f",&phi4);     //字符串转数字
    sscanf(argv[2],"%f",&phi1);     //字符串转数字
    _input_leg_angle(&leg_obj[0], phi4/180.f*PI, phi1/180.f*PI);
    leg_obj[0].resolve(&leg_obj[0]);
    rt_kprintf("PendulumLength = %f, PendulumRadian = %f\r\n", leg_obj[0].PendulumLength, leg_obj[0].PendulumRadian*180/PI);
    rt_kprintf("CoorC = [%f, %f], CoorB = [%f, %f], CoorD = [%f, %f]\r\n", leg_obj[0].CoorC[0], leg_obj[0].CoorC[1], leg_obj[0].CoorB[0], leg_obj[0].CoorB[1], leg_obj[0].CoorD[0], leg_obj[0].CoorD[1]);
    rt_kprintf("U2 = %f, U3 = %f\r\n", leg_obj[0].U2*180/PI, leg_obj[0].U3*180/PI);
}
MSH_CMD_EXPORT(input, leg test reslove input); */