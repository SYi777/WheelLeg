#ifndef __LEFT_LEG_H
#define __LEFT_LEG_H
#include "8009_CtrlApplications.h"
#include "pid.h"

#define M_PI 3.1415926
//pid 常量定义
#define PID_MAX_OUT 30.0f
#define PID_MAX_IOUT 5.0f
#define PID_P 2.0f
#define PID_I 0.0f
#define PID_D 9.0f
//左腿电机控制结构体定义，包含内容nopowerflag，tffout，当前角度，目标角度，pid控制结构体
typedef struct
{
	//来自传感器的值，用于初始化
	const fp32 *AnglePoint;
	
	//角度
	fp32 Now_AbsoluteAngle;
	fp32 Wanted_AbsoluteAngle;
	
	//PID
	pid_type_def angle_pid;
	
	//control
	int16_t Torque_out;
	uint8_t noPowerflag;
}LeftLegCtrl_s;



#endif
