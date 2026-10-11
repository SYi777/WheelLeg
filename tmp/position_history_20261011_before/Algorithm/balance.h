#ifndef __BALANCE_H
#define __BALANCE_H
#include "main.h"
#include "INS_task.h"
#include "VMC.h"

/* Sensor-coordinate equilibrium in radians. This is not the COM offset y [m]. */
#define THETA_E_RAD         (-0.065042f)
#define PITCH_GYRO_AXIS     0
#define LQR_LEG_SIGN        (1.0f)
#define LQR_PITCH_SIGN      (1.0f)
#define LQR_TP_SIGN         (1.0f)
#define LQR_WHEEL_SIGN      (-1.0f)
#define WHEEL_V_SIGN_L      (-1.0f)
#define WHEEL_V_SIGN_R      (1.0f)
#define WHEEL_RADIUS_M      0.06f
/* Existing RPM scale retained: motor rotor RPM -> output-wheel linear speed. */
#define WHEEL_RPM_TO_M_S    0.000327f
#define LQR_T_LIMIT_NM      5.0f
#define LQR_TP_LIMIT_NMM    13000.0f
#define LQR_X_LIMIT_M       0.3f
/* Negative feedback on v_R-v_L, expressed in MODEL forward wheel torque. */
#define WHEEL_DIFF_KD       1.0f   /* N*m / (m/s), initial damping gain */
#define WHEEL_DIFF_LIMIT_NM 0.8f

typedef struct
{
    float theta_rad, dtheta_rad_s;
    float x_m, v_m_s;
    float phi_rad, dphi_rad_s;
    /* Driver-facing wheel command; convert with LQR_WHEEL_SIGN for model T. */
    float T_Nm;
    float Tp_Nmm;
} balance_t;

typedef struct
{
    balance_t common;             /* Symmetric longitudinal state, one x integral. */
    float v_raw_L_m_s, v_raw_R_m_s;
    float v_L_m_s, v_R_m_s;
    float T_common_model_Nm, Tp_common_model_Nm;
    float T_diff_model_Nm;        /* +left, -right before driver sign conversion. */
    float wheel_terms_Nm[6], hip_terms_Nm[6];
} balance_pair_t;

float Balance_WheelVelocity(float rpm, float rpm_sign, const vmc_t *vmc,
                            const INS_t *ins);
void Balance_PairUpdate(balance_pair_t *pair, balance_t *left, balance_t *right,
                        const vmc_t *vmc_left, const vmc_t *vmc_right,
                        const INS_t *ins, float rpm_left, float rpm_right, float dt);
void Balance_PairReset(balance_pair_t *pair, balance_t *left, balance_t *right);
#endif
