/**
*@file Leg_Tasks.h
*@brief 左右腿控制任务及横滚、防劈叉接口
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/9/30
**/
#ifndef __LEG_TASKS_H
#define __LEG_TASKS_H
#include "8009_CtrlApplications.h"
#include "DM_motor.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "pid.h"

#define M_PI 3.1415926f
// 安全角度区间[rad]
#define MAX_DELTA (M_PI-0.2f)
#define MIN_DELTA (M_PI/2-0.1f)
// 角度环 PID 常量
#define PID_MAX_OUT 30.0f
#define PID_MAX_IOUT 5.0f
#define PID_P 25.0f
#define PID_I 0.0f
#define PID_D 10.0f
// 左腿控制结构体: 电机角度反馈、目标角度与角度环 PID
typedef struct
{
	// 电机角度反馈指针, 初始化时绑定
	const fp32 *AnglePoint;
	
	// 当前与目标绝对角度[rad]
	fp32 Now_AbsoluteAngle;
	fp32 Wanted_AbsoluteAngle;
	
	// 角度环 PID
	pid_type_def angle_pid;
	
	// 力矩输出[N·m] 与掉电标志
	int16_t Torque_out;
	uint8_t noPowerflag;
}LeftLegCtrl_s;

void LLeg_task(void *argument);
void RLeg_task(void *argument);
uint8_t Gap_check(LeftLegCtrl_s *c1,LeftLegCtrl_s *c2);
void Left_leg_init(void);
void Update_left_leg(LeftLegCtrl_s *c);
float Roll_Loop(float roll_set, float roll, float roll_gyro);
float AntiSplit_Loop(float theta_rel_body_L_rad, float theta_rel_body_R_rad);

#endif
