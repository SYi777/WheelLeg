/**
*@file 8009_Ctrl_Applications.c
*@brief 对逐个8009的电机进行功能例化
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-5
**/
#include "8009_CtrlApplications.h"

DM_Motor DM8009_Ctrl[4];

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
    //cmd清零
    motor->cmd.p_des = 0;
    motor->cmd.v_des = 0;
    motor->cmd.kp    = 0;
    motor->cmd.kd    = 0;
    motor->cmd.t_ff  = 0;
    //fb清零
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
//不包含发送命令，由其他函数统一发送
// MIT力矩环
void Set_LeftOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[0],0.0f,0.0f,0.0f,0.0f,t_ff);
}
void Set_LeftTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[1],0.0f,0.0f,0.0f,0.0f,t_ff);
}
void Set_RightOneTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[2],0.0f,0.0f,0.0f,0.0f,t_ff);
}
void Set_RightTwoTorqueMIT(FDCAN_HandleTypeDef *hfdcan,DM_Motor *g_motor,float t_ff)
{
	DM_MITCtrl(&DM8009_Ctrl[3],0.0f,0.0f,0.0f,0.0f,t_ff);
}
//MIT位置环
void Set_LeftOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[0], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
// 左后
void Set_LeftTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[1], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
// 右前
void Set_RightOnePositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[2], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
// 右后
void Set_RightTwoPositionMIT(FDCAN_HandleTypeDef *hfdcan, DM_Motor *g_motor, float pos)
{
    DM_MITCtrl(&DM8009_Ctrl[3], pos, 0.0f, 10.0f, 2.0f, 0.0f);
}
