/**
 * @file RobotState.h
 * @brief 轮腿整车状态机
 * @Auther Jing Yuhang  jyh16213839418@163.com
 * @version 0.4
 * @date 2026-10-13
 *
 */
#ifndef __ROBOTSTATE_H
#define __ROBOTSTATE_H

#include "struct_typedef.h"

enum RobotState_e{						//整车模式枚举
    RobotState_e_Powerless=0,     		//无力模式

    RobotState_e_CommonCar=3,     		//底盘不动模式

    RobotState_e_GimbalCar=1,        	//底盘跟随云台
                                
    RobotState_e_Spinner=2,     		//小陀螺
};

void RobotTotalState_Task(void *argument);

#endif
