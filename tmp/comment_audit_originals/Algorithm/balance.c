/**
*@file balance.c
*@brief 单腿连续 LQR 平衡状态与力矩计算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/11
*
*左右腿分别计算角度状态，轮速和位移由左腿任务共享。
**/
#include "balance.h"
#include <string.h>

// 单侧连续 LQR: 各腿使用自己的 theta/theta_dot，共享车轮 x/v
// Q=diag(10,1,50,10,800,1)，R=diag(20,1)，模型输出 u=-K*e
static const float K[2][6] = {
    {-9.4389f, -1.2684f, -1.3967f, -1.6995f, 4.7310f, 0.8119f},
    {14.3380f,  2.1396f,  3.3143f,  3.7405f,25.7565f, 2.2509f}
};

// 将轮电机 RPM 近似换算为轮心线速度
float Balance_WheelVelocity(float rpm, float rpm_sign)
{
    return rpm * WHEEL_RPM_TO_M_S * rpm_sign;
}

/**
 * @brief  更新左右轮共享速度与轮心位移
 * @param  sh  左右腿共享的轮速和位移状态
 * @param  rpm_left  左轮转速 [RPM]
 * @param  rpm_right  右轮转速 [RPM]
 * @param  dt  本次控制周期 [s]
 * @return 无
 *
 * 仅左腿任务调用，右腿任务读取更新结果。
 */
void Balance_SharedUpdate(balance_shared_t *sh, float rpm_left, float rpm_right,
                          float dt)
{
    sh->v_L_m_s = Balance_WheelVelocity(rpm_left, WHEEL_V_SIGN_L);
    sh->v_R_m_s = Balance_WheelVelocity(rpm_right, WHEEL_V_SIGN_R);
    sh->v_m_s = 0.5f * (sh->v_L_m_s + sh->v_R_m_s);

    // x 为轮心位移的积分，保持 x_dot=v；控制复位时清零原点
    sh->x_m += sh->v_m_s * dt;
}

// 清零本腿平衡状态
void Balance_Reset(balance_t *bal)
{
    memset(bal, 0, sizeof(*bal));
}

/**
 * @brief  按本腿角度及共享轮速计算连续 LQR 力矩
 * @param  bal  输出的本腿平衡状态与力矩
 * @param  sh  左右腿共享的轮速和位移
 * @param  vmc  本腿 VMC 状态
 * @param  ins  机身姿态与角速度
 * @return 无
 *
 * 状态顺序为 theta、theta_dot、x、x_dot、phi、phi_dot；轮输出为 N*m，髋输出为 N*mm。
 */
void Balance_Update(balance_t *bal, const balance_shared_t *sh,
                    const vmc_t *vmc, const INS_t *ins)
{
    // 机身姿态与腿角；FK 局部 +x 朝车头、+y 朝下
    bal->phi_rad = LQR_PITCH_SIGN * ins->Pitch;
    bal->dphi_rad_s = LQR_PITCH_SIGN * ins->Gyro[PITCH_GYRO_AXIS];
    bal->theta_rad = LQR_LEG_SIGN * vmc->theta_rel_body_rad - bal->phi_rad;
    bal->dtheta_rad_s = LQR_LEG_SIGN * vmc->theta_rel_body_dot_rad_s - bal->dphi_rad_s;

    // 两腿共用轮心位移和平均轮速
    bal->x_m = sh->x_m;
    bal->v_m_s = sh->v_m_s;

    // 状态误差顺序: theta, theta_dot, x, x_dot, phi, phi_dot
    float state_error[6] = {
        sh->theta_avg_rad - THETA_E_RAD,      // 原来是 bal->theta_rad - THETA_E_RAD
        sh->dtheta_avg_rad_s,                 // 原来是 bal->dtheta_rad_s
        bal->x_m,
        bal->v_m_s,
        bal->phi_rad,
        bal->dphi_rad_s,
    };

    // 连续 LQR: 第一行轮力矩，第二行髋力矩，单位均为 N*m
    float wheel_torque_Nm = 0.0f;
    float hip_torque_Nm = 0.0f;
    for (int j = 0; j < 6; j++)
    {
        wheel_torque_Nm -= K[0][j] * state_error[j];
        hip_torque_Nm -= K[1][j] * state_error[j];
    }

    // 模型力矩转驱动方向，再分别限幅
    wheel_torque_Nm *= LQR_WHEEL_SIGN;
    hip_torque_Nm *= LQR_TP_SIGN;
    if (wheel_torque_Nm >  LQR_T_LIMIT_NM) wheel_torque_Nm =  LQR_T_LIMIT_NM;
    if (wheel_torque_Nm < -LQR_T_LIMIT_NM) wheel_torque_Nm = -LQR_T_LIMIT_NM;

    // VMC 髋力矩使用 N*mm
    float hip_torque_Nmm = hip_torque_Nm * 1000.0f;
    if (hip_torque_Nmm >  LQR_TP_LIMIT_NMM) hip_torque_Nmm =  LQR_TP_LIMIT_NMM;
    if (hip_torque_Nmm < -LQR_TP_LIMIT_NMM) hip_torque_Nmm = -LQR_TP_LIMIT_NMM;

    bal->T_Nm = wheel_torque_Nm;
    bal->Tp_Nmm = hip_torque_Nmm;
}
