#include "balance.h"

/* Continuous-time LQR for [theta, theta_dot, x, x_dot, phi, phi_dot].
 * Q=diag(10,1,50,10,800,1), R=diag(20,1).
 * Row 0 outputs wheel T [N*m]; row 1 outputs hip T_p [N*m].
 * Model control law: u = u_e - K*(xi-xi_e), with u_e = 0. */
static const float K[2][6] = {
    {-9.4389f, -1.2684f, -1.3967f, -1.6995f,  4.7310f, 0.8119f},
    {14.3380f,  2.1396f,  3.3143f,  3.7405f, 25.7565f, 2.2509f}
};

void Balance_Update(balance_t *bal, const vmc_t *vmc, const INS_t *ins,
                    float v_wheel_m_s, float dt)
{
    /* 1. 状态合成 (模型约定, 见 VMC.h 头注: theta = theta_rel_body ? phi) */
    bal->phi_rad     = LQR_PITCH_SIGN * ins->Pitch;
    bal->dphi_rad_s  = LQR_PITCH_SIGN * ins->Gyro[PITCH_GYRO_AXIS];
    bal->theta_rad    = vmc->theta_rel_body_rad - bal->phi_rad;
    bal->dtheta_rad_s = vmc->theta_rel_body_dot_rad_s - bal->dphi_rad_s;
    bal->v_m_s = v_wheel_m_s;
    bal->x_m  += bal->v_m_s * dt;       /* 位移积分 */
    if (bal->x_m >  0.3f) bal->x_m =  0.3f;   /* 限幅防漂移偏置轮扭矩 */
    if (bal->x_m < -0.3f) bal->x_m = -0.3f;

    /* 2. u = ?K·(ξ?ξ_e) */
    float e[6] = {
        bal->theta_rad - THETA_E_RAD,
        bal->dtheta_rad_s,
        bal->x_m,
        bal->v_m_s,
        bal->phi_rad,
        bal->dphi_rad_s,
    };
    float T_Nm  = 0.0f, Tp_Nm = 0.0f;
    for (int j = 0; j < 6; j++)
    {
        T_Nm  -= K[0][j] * e[j];
        Tp_Nm -= K[1][j] * e[j];
    }

    /* 3. 符号核对 + 限幅 */
    T_Nm  *= LQR_WHEEL_SIGN;
    Tp_Nm *= LQR_TP_SIGN;
    if (T_Nm  >  LQR_T_LIMIT_NM)   T_Nm  =  LQR_T_LIMIT_NM;
    if (T_Nm  < -LQR_T_LIMIT_NM)   T_Nm  = -LQR_T_LIMIT_NM;
    bal->Tp_Nmm = Tp_Nm * 1000.0f;                 /* N·m → N·mm */
    if (bal->Tp_Nmm >  LQR_TP_LIMIT_NMM) bal->Tp_Nmm =  LQR_TP_LIMIT_NMM;
    if (bal->Tp_Nmm < -LQR_TP_LIMIT_NMM) bal->Tp_Nmm = -LQR_TP_LIMIT_NMM;
    bal->T_Nm = T_Nm;
}
