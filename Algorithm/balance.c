#include "balance.h"
#include <string.h>

/* 单侧纵向 LQR 增益 K: 每条腿用各自的 theta/theta_dot 加共享的 x/v.
 * Q=diag(10,1,50,10,800,1), R=diag(20,1); u_model=-K*e.
 * 双轮反向(差速)运动需要自己的阻尼, 不属于这个 LQR 模型. */
static const float K[2][6] = {
    {-9.4389f, -1.2684f, -1.3967f, -1.6995f, 4.7310f, 0.8119f},
    {14.3380f,  2.1396f,  3.3143f,  3.7405f,25.7565f, 2.2509f}
};

static float limit(float value, float bound)
{
    if (value > bound) return bound;
    if (value < -bound) return -bound;
    return value;
}

float Balance_WheelVelocity(float rpm, float rpm_sign, const vmc_t *vmc,
                            const INS_t *ins)
{
    /* FK 回路是传动等效, 不是真实连杆布局.
     * 定子(车身)也在动时, 相对转子 RPM 只是轮世界速度的近似.
     * 动力学里的 x 是轮心位置, 不是髋位置. */
    (void)vmc;
    (void)ins;
    return rpm * WHEEL_RPM_TO_M_S * rpm_sign;
}

void Balance_SharedUpdate(balance_shared_t *sh, float rpm_left, float rpm_right,
                          float dt)
{
    /* 整车共享 x/v: 单积分器, 由左腿任务独占调用, 右腿任务只读 */
    sh->v_L_m_s = Balance_WheelVelocity(rpm_left, WHEEL_V_SIGN_L, 0, 0);
    sh->v_R_m_s = Balance_WheelVelocity(rpm_right, WHEEL_V_SIGN_R, 0, 0);
    sh->v_m_s = 0.5f * (sh->v_L_m_s + sh->v_R_m_s);
    /* x 是实测位移的单积分, 不是执行器积分器.
     * 钳位 x 会破坏 x_dot=v, 并隐藏 LQR 视角下的持续平移.
     * 指令饱和在下面做; 控制复位时原点清零. */
    sh->x_m += sh->v_m_s * dt;
}

void Balance_Reset(balance_t *bal)
{
    memset(bal, 0, sizeof(*bal));
}

void Balance_Update(balance_t *bal, const balance_shared_t *sh,
                    const vmc_t *vmc, const INS_t *ins)
{
    /* 每腿独立角度 + 共享 x/v + 共享机身姿态 */
    bal->phi_rad = LQR_PITCH_SIGN * ins->Pitch;
    bal->dphi_rad_s = LQR_PITCH_SIGN * ins->Gyro[PITCH_GYRO_AXIS];
    /* FK 局部 +x 朝车头、+y 朝下；模型 theta 是世界系 W->H 的前倾角。 */
    bal->theta_rad = LQR_LEG_SIGN * vmc->theta_rel_body_rad - bal->phi_rad;
    bal->dtheta_rad_s = LQR_LEG_SIGN * vmc->theta_rel_body_dot_rad_s - bal->dphi_rad_s;
    bal->x_m = sh->x_m;
    bal->v_m_s = sh->v_m_s;

    float e[6] = {bal->theta_rad - THETA_E_RAD, bal->dtheta_rad_s,
                  sh->x_m, sh->v_m_s, bal->phi_rad, bal->dphi_rad_s};
    float T = 0.0f, Tp = 0.0f;
    for (int j = 0; j < 6; ++j)
    {
        T  -= K[0][j] * e[j];
        Tp -= K[1][j] * e[j];
    }
    T  = LQR_WHEEL_SIGN * T;
    Tp = LQR_TP_SIGN * Tp;
    bal->T_Nm   = limit(T, LQR_T_LIMIT_NM);
    bal->Tp_Nmm = limit(Tp * 1000.0f, LQR_TP_LIMIT_NMM);
}
