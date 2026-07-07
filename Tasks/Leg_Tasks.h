#ifndef __LEG_TASKS_H
#define __LEG_TASKS_H
#include "8009_CtrlApplications.h"
#include "DM_motor.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "pid.h"

#define M_PI 3.1415926f
//安全区间定义
#define MAX_DELTA (M_PI-0.2f)
#define MIN_DELTA (M_PI/2-0.1f)
//pid 常量定义
#define PID_MAX_OUT 30.0f
#define PID_MAX_IOUT 5.0f
#define PID_P 25.0f
#define PID_I 0.0f
#define PID_D 10.0f
//左腿电机控制结构体定义，包含内容nopowerflag，tffout，当前角度，目标角度，pid控制结构体
typedef struct
{
	//来自电机的值，用于初始化
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

void LLeg_task(void *argument);
void RLeg_task(void *argument);
uint8_t Gap_check(LeftLegCtrl_s *c1,LeftLegCtrl_s *c2);
void Left_leg_init(void);
void Update_left_leg(LeftLegCtrl_s *c);

#endif
