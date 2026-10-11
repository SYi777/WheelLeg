#include "balance.h"
#include <string.h>

/* Single-side longitudinal K: apply once to the symmetric state, then to BOTH sides.
 * Q=diag(10,1,50,10,800,1), R=diag(20,1); u_model=-K*e.
 * The antisymmetric wheel motion needs its own damping; it is not this LQR model. */
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
    /* The FK loop is a TRANSMISSION EQUIVALENT, not the physical link layout.
     * Do not use virtual th1/th2 to infer stator rotation. Preserve the verified
     * encoder speed convention until the real serial transmission is mapped.
     * Relative rotor RPM is an approximation to world wheel speed when the
     * stator moves. x in the dynamics is wheel-center position, not hip position. */
    (void)vmc;
    (void)ins;
    return rpm * WHEEL_RPM_TO_M_S * rpm_sign;
}

void Balance_PairReset(balance_pair_t *pair, balance_t *left, balance_t *right)
{
    memset(pair, 0, sizeof(*pair));
    memset(left, 0, sizeof(*left));
    memset(right, 0, sizeof(*right));
}

void Balance_PairUpdate(balance_pair_t *pair, balance_t *left, balance_t *right,
                        const vmc_t *vl, const vmc_t *vr, const INS_t *ins,
                        float rpm_left, float rpm_right, float dt)
{
    balance_t *c = &pair->common;
    c->phi_rad = LQR_PITCH_SIGN * ins->Pitch;
    c->dphi_rad_s = LQR_PITCH_SIGN * ins->Gyro[PITCH_GYRO_AXIS];
    left->theta_rad = LQR_LEG_SIGN * vl->theta_rel_body_rad - c->phi_rad;
    right->theta_rad = LQR_LEG_SIGN * vr->theta_rel_body_rad - c->phi_rad;
    left->dtheta_rad_s = LQR_LEG_SIGN * vl->theta_rel_body_dot_rad_s - c->dphi_rad_s;
    right->dtheta_rad_s = LQR_LEG_SIGN * vr->theta_rel_body_dot_rad_s - c->dphi_rad_s;
    pair->v_raw_L_m_s = rpm_left * WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_L;
    pair->v_raw_R_m_s = rpm_right * WHEEL_RPM_TO_M_S * WHEEL_V_SIGN_R;
    pair->v_L_m_s = Balance_WheelVelocity(rpm_left, WHEEL_V_SIGN_L, vl, ins);
    pair->v_R_m_s = Balance_WheelVelocity(rpm_right, WHEEL_V_SIGN_R, vr, ins);
    c->theta_rad = 0.5f * (left->theta_rad + right->theta_rad);
    c->dtheta_rad_s = 0.5f * (left->dtheta_rad_s + right->dtheta_rad_s);
    c->v_m_s = 0.5f * (pair->v_L_m_s + pair->v_R_m_s);
    /* x is measured displacement, not an actuator integrator.
     * Clamping x breaks x_dot=v and hides continued translation from LQR.
     * Keep command saturation below; reset the origin on control reset. */
    c->x_m += c->v_m_s * dt;

    float e[6] = {c->theta_rad - THETA_E_RAD, c->dtheta_rad_s,
                  c->x_m, c->v_m_s, c->phi_rad, c->dphi_rad_s};
    float T = 0.0f, Tp = 0.0f;
    for (int j = 0; j < 6; ++j)
    {
        pair->wheel_terms_Nm[j] = -K[0][j] * e[j];
        pair->hip_terms_Nm[j] = -K[1][j] * e[j];
        T += pair->wheel_terms_Nm[j];
        Tp += pair->hip_terms_Nm[j];
    }
    pair->T_common_model_Nm = limit(T, LQR_T_LIMIT_NM);
    pair->Tp_common_model_Nm = limit(Tp, LQR_TP_LIMIT_NMM / 1000.0f);
    pair->T_diff_model_Nm = limit(WHEEL_DIFF_KD * (pair->v_R_m_s - pair->v_L_m_s),
                                 WHEEL_DIFF_LIMIT_NM);
    left->T_Nm = LQR_WHEEL_SIGN * limit(pair->T_common_model_Nm + pair->T_diff_model_Nm,
                                       LQR_T_LIMIT_NM);
    right->T_Nm = LQR_WHEEL_SIGN * limit(pair->T_common_model_Nm - pair->T_diff_model_Nm,
                                        LQR_T_LIMIT_NM);
    left->Tp_Nmm = right->Tp_Nmm = LQR_TP_SIGN * pair->Tp_common_model_Nm * 1000.0f;
    left->x_m = right->x_m = c->x_m;
    left->v_m_s = pair->v_L_m_s;
    right->v_m_s = pair->v_R_m_s;
    left->phi_rad = right->phi_rad = c->phi_rad;
    left->dphi_rad_s = right->dphi_rad_s = c->dphi_rad_s;
    c->T_Nm = LQR_WHEEL_SIGN * pair->T_common_model_Nm;
    c->Tp_Nmm = left->Tp_Nmm;
}
