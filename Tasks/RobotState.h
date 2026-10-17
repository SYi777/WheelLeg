/**
*@file RobotState.h
*@brief 轮腿整车状态机
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-10-13
*
*定义整车模式枚举与状态查询接口。
**/
#ifndef __ROBOTSTATE_H
#define __ROBOTSTATE_H

#include "struct_typedef.h"

enum RobotState_e{						// 整车模式枚举
    RobotState_e_Powerless=0,     		// 无力模式: 不输出力矩

    RobotState_e_CommonCar=3,     		// 底盘不动模式

    RobotState_e_GimbalCar=1,        	// 底盘跟随云台
                                
    RobotState_e_Spinner=2,     		// 小陀螺模式
};

void RobotTotalState_Task(void *argument);

const enum RobotState_e * getRobotPresentMode(void);

#endif
