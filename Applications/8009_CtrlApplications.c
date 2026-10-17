/**
*@file 8009_CtrlApplications.c
*@brief 对逐个 8009 电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
*
*各函数按固定索引操作 DM8009_Ctrl[], 形参 hfdcan/g_motor 当前未使用;
*仅更新 MIT 指令, 不发送 CAN。
**/
#include "8009_CtrlApplications.h"

DM_Motor DM8009_Ctrl[4];

/**
 * @brief  初始化一台 DM8009 的控制与反馈状态
 * @param  motor  电机控制对象
 * @param  can_id  该电机的 CAN ID
 * @return 无
 *
 * 仅清零软件状态, 不发送 CAN 报文。
 */
void DM_8009Motor_Init(DM_Motor *motor, uint8_t can_id)
{
    if (motor == NULL) return;
    motor->can_id          = can_id;
    motor->online          = 0;
    motor->rx_cnt          = 0;
    motor->tx_cnt          = 0;
    motor->last_fb_tick    = 0;
    motor->last_cmd_tick   = 0;
    motor->last_tx_status  = HAL_OK;
    // 清零控制指令
    motor->cmd.p_des = 0;
    motor->cmd.v_des = 0;
    motor->cmd.kp    = 0;
    motor->cmd.kd    = 0;
    motor->cmd.t_ff  = 0;
    // 清零反馈状态
    motor->fb.pos_raw   = 0;
    motor->fb.vel_raw   = 0;
    motor->fb.tor_raw   = 0;
    motor->fb.err       = 0;
    motor->fb.motor_id  = 0;
    motor->fb.t_mos     = 0;
    motor->fb.t_rotor   = 0;
    motor->fb.position  = 0.0f;
    motor->fb.velocity  = 0.0f;
    motor->fb.torque    = 0.0f;
    motor->fb.enabled   = 0;
    motor->fb.fault     = 0;
    motor->fb.fault_code = DM_FAULT_NONE;
}
/**
 * @brief  设置左前关节 (关节 id=1) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左后方, 固定操作 DM8009_Ctrl[0]; 仅更新指令, 不发送 CAN。
 */
void Set_LeftOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[0],0.0f,0.0f,0.0f,0.0f,t_ff);
}
/**
 * @brief  设置左后关节 (关节 id=2) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于左前方, 固定操作 DM8009_Ctrl[1]; 仅更新指令, 不发送 CAN。
 */
void Set_LeftTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[1],0.0f,0.0f,0.0f,0.0f,t_ff);
}
/**
 * @brief  设置右前关节 (关节 id=3) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右前方, 固定操作 DM8009_Ctrl[2]; 仅更新指令, 不发送 CAN。
 */
void Set_RightOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[2],0.0f,0.0f,0.0f,0.0f,t_ff);
}
/**
 * @brief  设置右后关节 (关节 id=4) 的 MIT 前馈力矩
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  t_ff  前馈力矩 [N*m]
 * @return 无
 *
 * 电机位于右后方, 固定操作 DM8009_Ctrl[3]; 仅更新指令, 不发送 CAN。
 */
void Set_RightTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[3],0.0f,0.0f,0.0f,0.0f,t_ff);
}
/**
 * @brief  设置左前关节 (关节 id=1) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于左后方, 固定操作 DM8009_Ctrl[0]; Kp=10, Kd=2, 不发送 CAN。
 */
void Set_LeftOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[0], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
/**
 * @brief  设置左后关节 (关节 id=2) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于左前方, 固定操作 DM8009_Ctrl[1]; Kp=10, Kd=2, 不发送 CAN。
 */
void Set_LeftTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[1], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
/**
 * @brief  设置右前关节 (关节 id=3) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于右前方, 固定操作 DM8009_Ctrl[2]; Kp=10, Kd=2, 不发送 CAN。
 */
void Set_RightOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[2], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
/**
 * @brief  设置右后关节 (关节 id=4) 的 MIT 目标角度
 * @param  hfdcan  未使用, 预留 CAN 句柄
 * @param  g_motor  未使用, 预留电机对象
 * @param  pos  目标角度 [rad]
 * @return 无
 *
 * 电机位于右后方, 固定操作 DM8009_Ctrl[3]; Kp=10, Kd=2, 不发送 CAN。
 */
void Set_RightTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[3], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
