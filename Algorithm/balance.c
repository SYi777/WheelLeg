/**
*@file balance.c
*@brief 对称 LQR 平衡状态更新与力矩计算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/11
*
*左腿任务先更新共享 x/v 与平均腿角, 再调 Balance_Update 算对称 LQR 输出;
*误差向量对双腿相同, 故双腿髋力矩同值、轮力矩同值(差速阻尼在任务层叠加)。
**/
#include "balance.h"
#include <string.h>

// 单侧纵向 LQR 增益 K: 作用于对称状态 (平均腿角 + 共享 x/v)
// Q=diag(10,1,50,10,800,1), R=diag(20,1); 模型输出 u=-K*e
// 第一行为轮力矩, 第二行为髋力矩
static const float K[2][6] = {
    {-9.4389f, -1.2684f, -1.3967f, -1.6995f, 4.7310f, 0.8119f},
    {14.3380f,  2.1396f,  3.3143f,  3.7405f,25.7565f, 2.2509f}
};

// 对称限幅
static float limit(float value, float bound)
{
    if (value > bound) return bound;
    if (value < -bound) return -bound;
    return value;
}

/**
 * @brief  电机转子 RPM 换算为轮心线速度
 * @param  rpm      电机转子转速 [RPM]
 * @param  rpm_sign 符号 (WHEEL_V_SIGN_L/R)
 * @return 轮心线速度 [m/s]
 */
float Balance_WheelVelocity(float rpm, float rpm_sign)
{
    return rpm * WHEEL_RPM_TO_M_S * rpm_sign;
}

/**
 * @brief  更新整车共享轮速与位移 (左腿任务独占调用)
 * @param  sh        共享状态
 * @param  rpm_left  左轮转速 [RPM]
 * @param  rpm_right 右轮转速 [RPM]
 * @param  dt        控制周期 [s]
 * @return 无
 */
void Balance_SharedUpdate(balance_shared_t *sh, float rpm_left, float rpm_right,
                          float dt)
{
    sh->v_L_m_s = Balance_WheelVelocity(rpm_left, WHEEL_V_SIGN_L);
    sh->v_R_m_s = Balance_WheelVelocity(rpm_right, WHEEL_V_SIGN_R);
    sh->v_m_s = 0.5f * (sh->v_L_m_s + sh->v_R_m_s);
    /* x 为实测位移积分, 不钳位 (钳位会破坏 x_dot=v); 控制复位时原点清零 */
    sh->x_m += sh->v_m_s * dt;
}

/**
 * @brief  复位单腿平衡状态
 * @param  bal  单腿状态
 * @return 无
 */
void Balance_Reset(balance_t *bal)
{
    memset(bal, 0, sizeof(*bal));
}

/**
 * @brief  对称 LQR: 平均腿角 + 共享 x/v -> 本腿轮力矩与髋力矩
 * @param  bal  本腿状态 (输出 T_Nm/Tp_Nmm; 误差向量对称, 双腿输出同值)
 * @param  sh   共享状态 (平均腿角与 x/v)
 * @param  vmc  本腿 VMC 状态
 * @param  ins  姿态数据
 * @return 无
 */
void Balance_Update(balance_t *bal, const balance_shared_t *sh,
                    const vmc_t *vmc, const INS_t *ins)
{
    // 姿态与本腿世界系腿角 (机体系腿角 - 机身俯仰)
    bal->phi_rad = LQR_PITCH_SIGN * ins->Pitch;
    bal->dphi_rad_s = LQR_PITCH_SIGN * ins->Gyro[PITCH_GYRO_AXIS];
    bal->theta_rad = LQR_LEG_SIGN * vmc->theta_rel_body_rad - bal->phi_rad;
    bal->dtheta_rad_s = LQR_LEG_SIGN * vmc->theta_rel_body_dot_rad_s - bal->dphi_rad_s;

    // 共享位移与速度
    bal->x_m = sh->x_m;
    bal->v_m_s = sh->v_m_s;

    // LQR 状态顺序: theta, theta_dot, x, x_dot, phi, phi_dot
    float state_error[6] = {
        sh->theta_avg_rad - THETA_E_RAD,
        sh->dtheta_avg_rad_s,
        bal->x_m,
        bal->v_m_s,
        bal->phi_rad,
        bal->dphi_rad_s,
    };

    // 第一行轮力矩, 第二行髋力矩, 单位均为 N*m
    float wheel_torque_Nm = 0.0f;
    float hip_torque_Nm = 0.0f;
    for (int j = 0; j < 6; j++)
    {
        wheel_torque_Nm -= K[0][j] * state_error[j];
        hip_torque_Nm -= K[1][j] * state_error[j];
    }

    // 模型输出 -> 驱动系符号换算 + 限幅
    wheel_torque_Nm *= LQR_WHEEL_SIGN;
    hip_torque_Nm *= LQR_TP_SIGN;
    if (wheel_torque_Nm >  LQR_T_LIMIT_NM) wheel_torque_Nm =  LQR_T_LIMIT_NM;
    if (wheel_torque_Nm < -LQR_T_LIMIT_NM) wheel_torque_Nm = -LQR_T_LIMIT_NM;

    float hip_torque_Nmm = hip_torque_Nm * 1000.0f;
    if (hip_torque_Nmm >  LQR_TP_LIMIT_NMM) hip_torque_Nmm =  LQR_TP_LIMIT_NMM;
    if (hip_torque_Nmm < -LQR_TP_LIMIT_NMM) hip_torque_Nmm = -LQR_TP_LIMIT_NMM;

    bal->T_Nm = wheel_torque_Nm;
    bal->Tp_Nmm = hip_torque_Nmm;
}
