/**
*@file 8009_CtrlApplications.h
*@brief 对逐个 8009 电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*从机器后方俯视, 各电机 CAN ID 与 MASTER ID 的对应关系:
*左后 CAN ID:0x001 MASTER ID:0x011
*左前 CAN ID:0x002 MASTER ID:0x012
*右后 CAN ID:0x003 MASTER ID:0x013
*右前 CAN ID:0x004 MASTER ID:0x014
*DM8009_Ctrl[4] 按上述顺序依次定义。
*
*8009 电机逆时针为正; 左侧以车前方为零界线。
*关节编号按并联腿等效模型描述, 只是电机前后交换了。
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
/**
 * @brief  初始化一台 DM8009 的控制与反馈状态
 * @param  g_motor  电机控制对象
 * @param  can_id  该电机的 CAN ID
 * @return 无
 */
void DM_8009Motor_Init(DM_Motor *g_motor, uint8_t can_id);
// 并联腿等效模型下的 MIT 前馈力矩接口
/**
 * @brief  设置左前关节 (关节 id=1) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左后方, 固定操作 DM8009_Ctrl[0]; 仅更新指令, 不发送 CAN。
 */
void Set_LeftOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
/**
 * @brief  设置左后关节 (关节 id=2) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左前方, 固定操作 DM8009_Ctrl[1]; 仅更新指令, 不发送 CAN。
 */
void Set_LeftTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
/**
 * @brief  设置右前关节 (关节 id=3) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右前方, 固定操作 DM8009_Ctrl[2]; 仅更新指令, 不发送 CAN。
 */
void Set_RightOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);
/**
 * @brief  设置右后关节 (关节 id=4) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右后方, 固定操作 DM8009_Ctrl[3]; 仅更新指令, 不发送 CAN。
 */
void Set_RightTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff);

/**
 * @brief  位置环接口: 以 MIT 位置模式设置各关节目标角度
 *
 * 按 DM8009_Ctrl[0..3] 顺序对应上述四个关节; Kp=10, Kd=2, 仅更新指令, 不发送 CAN。
 */
void Set_LeftOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_LeftTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_RightOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);
void Set_RightTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos);

#endif
