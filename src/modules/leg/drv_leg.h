/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#ifndef _DRV_LEG_H
#define _DRV_LEG_H

#include "rtthread.h"

enum leg_state_e
{
	LEG_NORMAL	= 0,
	LEG_ERROR	= 1
};

typedef struct leg_obj
{
	/*单位m*/
	float l1;  // l4=l1
	float l2; // l3=l2
	float moto_distance; //电机间距
	float moto_distance_half; // moto_distance/2.0f
	
	/*关节电机弧度*/
	float phi1;
	float phi4;
    /*膝关节弧度*/
    float phi2;
	
	/*极限值*/
	float phi1_min; // PI/2
	float phi4_max; // PI/2
	/*杆状态*/
	int8_t leg_state;
	
	/*倒立摆长度*/
	float PendulumLength;
	/*倒立摆角度*/
	float PendulumRadian;
	/*倒立摆坐标*/
	float CoorC[2];
	/*第二象限节点坐标*/
	float CoorB[2];
	float U2;
	/*第二象限节点坐标*/
	float CoorD[2];
	float U3;

	void (*resolve)(struct leg_obj *leg);
	void (*VMC_cal)(struct leg_obj *leg, float *FT, float *Tmotor);
	int8_t (*input_leg_angle)(struct leg_obj *leg, float phi3, float phi2);
}leg_obj_t;

typedef struct
{
	/*单位mm*/
	float l1;  // l4=l1
	float l2; // l3=l2
	float moto_distance; //电机间距
    /* 后续可能需要加入更多参数 */
}leg_config_t;

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t *leg_register(leg_config_t *config);

#endif //_DRV_LEG_H
