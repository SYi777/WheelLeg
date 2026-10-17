/**
*@file 8009_Ctrl_Applications.h
*@brief 对逐个8009的电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*从机器后面俯视
*左后CAN ID:0x001 MASTER ID:0x011
*左前CAN ID:0x002 MASTER ID:0x012
*右后CAN ID:0x003 MASTER ID:0x013
*右前CAN ID:0x004 MASTER ID:0x014
*定义为DM8009_Ctrl[4],依次
*
*8009电机逆时针为正，左边取车前方为零界线
*描述等效在并联腿情况下，只是电机前后交换了
*
**/
#ifndef __8009_CTRLAPPLICATIONS_H
#define __8009_CTRLAPPLICATIONS_H
#include "DM_motor.h"

#define LeftFrontCAN_ID 0x01
#define LeftBehindCAN_ID 0x02
#define RightFrontCAN_ID 0x03
#define RightBehindCAN_ID 0x04

#define LeftFrontMASTER_ID 0x11
#define LeftBehindMASTER_ID 0x12
#define RightFrontMASTER_ID 0x13
#define RightBehindMASTER_ID 0x14

extern DM_Motor DM8009_Ctrl[4];
//初始化电机
void DM_8009Motor_Init(DM_Motor *g_motor, uint8_t can_id);
//并联腿等效模型下
//设置左前关节，关节id为1，电机位于左后方
void Set_LeftOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
//设置左后关节，关节id为2，电机位于左前方
void Set_LeftTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
//设置右前关节，关节id为3，电机位于右前方
void Set_RightOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
//设置右后关节，关节id为4，电机位于右后方
void Set_RightTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);

// 位置环
void Set_LeftOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_LeftTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_RightOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_RightTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);

#endif
