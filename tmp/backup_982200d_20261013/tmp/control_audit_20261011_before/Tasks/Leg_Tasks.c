#include "Leg_Tasks.h"
#include "Remote_control.h"
#include "VMC.h"
#include "3508_CtrlApplications.h"
#include "balance.h"
#include "INS_task.h"
#include "cmsis_os2.h"

extern vmc_t vmc_left_s;
extern vmc_t vmc_right_s;

balance_t bal_left;
balance_t bal_right;

float Roll_Loop(float roll_set, float roll, float roll_gyro);
float AntiSplit_Loop(float theta_rel_body_L_rad, float theta_rel_body_R_rad);

/* 安全保护: 电机断联或正解失败 -> 禁止输出 */
static int leg_safety_ok(const DM_Motor *m0, const DM_Motor *m1, const vmc_t *vmc)
{
    uint32_t now = HAL_GetTick();
    if (m0->online == 0 || m1->online == 0) return 0;
    if (now - m0->last_fb_tick > DM_COMM_TIMEOUT_MS) return 0;
    if (now - m1->last_fb_tick > DM_COMM_TIMEOUT_MS) return 0;
    if (vmc->fk_s.fk.status != FK_OK) return 0;
    return 1;
}
static int flag = 0;
void LLeg_task(void *argument)
{
	osDelay(5000);

    DM_8009Motor_Init(&DM8009_Ctrl[0], LeftFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[0]);
    DM_8009Motor_Init(&DM8009_Ctrl[1], LeftBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[1]);
	for (int j = 0; j < 10; j++)
	{
		DM_Enable(&hfdcan1, &DM8009_Ctrl[0]);
		DM_Enable(&hfdcan1, &DM8009_Ctrl[1]);
		osDelay(2);
	}

    VMC_init(&vmc_left_s);

    while (1)
    {
        float joint_a_angle_rad = DM8009_Ctrl[0].fb.processed_pos;
        float joint_b_angle_rad  = DM8009_Ctrl[1].fb.processed_pos;

        VMC_StateUpdate(&vmc_left_s, joint_a_angle_rad, joint_b_angle_rad, 0.001f);

        if (leg_safety_ok(&DM8009_Ctrl[0], &DM8009_Ctrl[1], &vmc_left_s))
        {
            /* 达妙式纯控制: 横滚 + 防劈叉 + LQR, 每周期全通道输出 */
            float T_p_antisplit_Nmm = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad,
                                                     vmc_right_s.theta_rel_body_rad);
            vmc_left_s.F_roll_N = Roll_Loop(0.0f, INS.Roll, INS.Gyro[1]);

            float v_wheel_m_s = motor_measure_L.speed_rpm * 0.000327f * WHEEL_V_SIGN_L;
            Balance_Update(&bal_left, &vmc_left_s, &INS, v_wheel_m_s, 0.001f);
            vmc_left_s.T_p_Nmm = bal_left.Tp_Nmm - T_p_antisplit_Nmm;
            VMC_ForceCalc(&vmc_left_s, joint_a_angle_rad, joint_b_angle_rad);
            WheelTorque_Send(&hfdcan2, bal_left.T_Nm, bal_right.T_Nm);
        }
        else
        {
            /* 保护: 全零 + 复位 */
            vmc_left_s.fk_s.tau_a = vmc_left_s.fk_s.tau_b = 0.0f;
            vmc_left_s.T_p_Nmm = 0.0f;
            bal_left.T_Nm = 0.0f;
            bal_left.Tp_Nmm = 0.0f;
            bal_left.x_m = 0.0f;
            vmc_left_s.first_flag = 1;
            WheelTorque_Send(&hfdcan2, 0.0f, 0.0f);
        }

        /* 下发 (限幅为常规输出保护, 达妙同款) */
        float ta = vmc_left_s.fk_s.tau_a / 1000.0f;
        float tb = vmc_left_s.fk_s.tau_b / 1000.0f;
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;
        Set_LeftOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[0], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[0]);
        Set_LeftTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[1], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[1]);

        osDelay(1);
    }
}

void RLeg_task(void *argument)
{
    osDelay(5000);

    DM_8009Motor_Init(&DM8009_Ctrl[2], RightFrontCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[2]);
    DM_8009Motor_Init(&DM8009_Ctrl[3], RightBehindCAN_ID);
    DM_CtrlStop(&DM8009_Ctrl[3]);
	for (int j = 0; j < 10; j++)
	{
		DM_Enable(&hfdcan1, &DM8009_Ctrl[2]);
		DM_Enable(&hfdcan1, &DM8009_Ctrl[3]);
		osDelay(2);
	}

    VMC_init(&vmc_right_s);

    while (1)
    {
        float joint_a_angle_rad = -DM8009_Ctrl[2].fb.processed_pos;   /* 镜像: 进运动学前取反 */
        float joint_b_angle_rad  = -DM8009_Ctrl[3].fb.processed_pos;

        VMC_StateUpdate(&vmc_right_s, joint_a_angle_rad, joint_b_angle_rad, 0.001f);

        if (leg_safety_ok(&DM8009_Ctrl[2], &DM8009_Ctrl[3], &vmc_right_s))
        {
            float T_p_antisplit_Nmm = AntiSplit_Loop(vmc_left_s.theta_rel_body_rad,
                                                     vmc_right_s.theta_rel_body_rad);
            vmc_right_s.F_roll_N = -Roll_Loop(0.0f, INS.Roll, INS.Gyro[1]);

            float v_wheel_m_s = motor_measure_R.speed_rpm * 0.000327f * WHEEL_V_SIGN_R;
            Balance_Update(&bal_right, &vmc_right_s, &INS, v_wheel_m_s, 0.001f);
            vmc_right_s.T_p_Nmm = bal_right.Tp_Nmm + T_p_antisplit_Nmm;
            VMC_ForceCalc(&vmc_right_s, joint_a_angle_rad, joint_b_angle_rad);
        }
        else
        {
            vmc_right_s.fk_s.tau_a = vmc_right_s.fk_s.tau_b = 0.0f;
            vmc_right_s.T_p_Nmm = 0.0f;
            bal_right.T_Nm = 0.0f;
            bal_right.Tp_Nmm = 0.0f;
            bal_right.x_m = 0.0f;
            vmc_right_s.first_flag = 1;
        }

        float ta = -vmc_right_s.fk_s.tau_a / 1000.0f;   /* 出运动学后取反 */
        float tb = -vmc_right_s.fk_s.tau_b / 1000.0f;
        if (ta >  TORQUE_LIMIT) ta =  TORQUE_LIMIT;
        if (ta < -TORQUE_LIMIT) ta = -TORQUE_LIMIT;
        if (tb >  TORQUE_LIMIT) tb =  TORQUE_LIMIT;
        if (tb < -TORQUE_LIMIT) tb = -TORQUE_LIMIT;
        Set_RightOneTorqueMIT(&hfdcan1, &DM8009_Ctrl[2], ta);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[2]);
        Set_RightTwoTorqueMIT(&hfdcan1, &DM8009_Ctrl[3], tb);
        DM_SendCmd(&hfdcan1, &DM8009_Ctrl[3]);

        osDelay(1);
    }
}

/* 横滚环: Roll 误差 -> 两腿力差 (Roll loop: roll error -> leg force diff)
 * roll_set : 目标横滚角 [rad], 站立 = 0, 压弯时给非零
 * roll     : INS.Roll [rad]
 * roll_gyro: 横滚轴角速度 [rad/s]
 * 返回 ΔF [N], 左腿取正、右腿取负 */
float Roll_Loop(float roll_set, float roll, float roll_gyro)
{
    float out = ROLL_KP * (roll_set - roll) - ROLL_KD * roll_gyro;
    if (out >  ROLL_MAX_OUT) out =  ROLL_MAX_OUT;
    if (out < -ROLL_MAX_OUT) out = -ROLL_MAX_OUT;
    return out;
}

/* 防劈叉环: 两腿夹角差 -> 修正力矩 (Anti-split: leg angle diff -> corrective T_p_Nmm)
 * 返回 K*(theta_rel_body_L_rad - theta_rel_body_R_rad) [N*mm]; 左腿取负、右腿取正
 * 两腿同向一起摆时差=0, 不产生力矩; 前后劈叉时差≠0, 力矩把两腿拉回对称 */
float AntiSplit_Loop(float theta_rel_body_L_rad, float theta_rel_body_R_rad)
{
    float out = ANTI_SPLIT_KP * (theta_rel_body_L_rad - theta_rel_body_R_rad);
    if (out >  ANTI_SPLIT_MAX) out =  ANTI_SPLIT_MAX;
    if (out < -ANTI_SPLIT_MAX) out = -ANTI_SPLIT_MAX;
    return out;
}
