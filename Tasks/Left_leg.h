/**
*@file Left_leg.h
*@brief 左腿控制结构体与 PID 参数
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date
*
*定义单腿角度闭环所需的结构体与 PID 常量。
*注意: Leg_Tasks.h 中另有同名结构体定义, 两个头文件不可同时包含。
**/
#ifndef __LEFT_LEG_H
#define __LEFT_LEG_H
#include "8009_CtrlApplications.h"
#include "pid.h"

#define M_PI 3.1415926
// PID 参数
#define PID_MAX_OUT 30.0f
#define PID_MAX_IOUT 5.0f
#define PID_P 2.0f
#define PID_I 0.0f
#define PID_D 9.0f
// 左腿控制结构体: 角度反馈、目标角度、PID 控制器与输出
typedef struct
{
	// 输入: 传感器角度指针, 用于初始化
	const fp32 *AnglePoint;
	
	// 状态: 当前绝对角度与目标绝对角度
	fp32 Now_AbsoluteAngle;
	fp32 Wanted_AbsoluteAngle;
	
	// PID 控制器
	pid_type_def angle_pid;
	
	// 输出: 力矩指令与无力标志
	int16_t Torque_out;
	uint8_t noPowerflag;
}LeftLegCtrl_s;



#endif
