#ifndef __BALANCE_H
#define __BALANCE_H
#include "main.h"
#include "INS_task.h"
#include "VMC.h"

#define THETA_E_RAD         0.045542f   /* 平衡点腿角: 髋略前为正 (模型值, 与 K 同源 y=-0.065042) */
#define PITCH_GYRO_AXIS     0              /* M0 标定: 车绕左右轴点头看哪个 Gyro 轴 */

#define LQR_T_LIMIT_NM      5.0f           /* 轮扭矩限幅 [N·m] (与电机最大输出一致) */
#define LQR_TP_LIMIT_NMM    13000.0f        /* 髋力矩限幅 [N·mm] */

/* M2 符号核对宏 (推腿测试后定死):
 * LQR_TP_SIGN = +1: observed nose-down response matches the model.
 * LQR_WHEEL_SIGN = 第一步先置 0 (只测髋通道), 轮子方向标定后置 ±1
 * LQR_PITCH_SIGN = 模型 φ 与 INS.Pitch 同正(车头仰起 Pitch 为正)时 +1 */
#define LQR_TP_SIGN         (1.0f)
#define LQR_WHEEL_SIGN      ( -1.0f)
#define LQR_PITCH_SIGN      ( 1.0f)

#define WHEEL_RADIUS_M      0.06f           /* 与建模 R 一致 */
#define WHEEL_V_SIGN_L      (-1.0f)         /* M0 标定, 见文末说明 */
#define WHEEL_V_SIGN_R      (1.0f)         /* M0 标定, 见文末说明 */

typedef struct
{
    float theta_rad;        /* 模型 θ = theta_rel_body_rad ? phi  [rad] */
    float dtheta_rad_s;     /* 模型 θ? */
    float x_m;              /* 轮位移 [m] (轮速里程计未接, 暂 0) */
    float v_m_s;            /* 车速 [m/s] */
    float phi_rad;          /* 模型 φ = INS.Pitch [rad] */
    float dphi_rad_s;       /* 模型 φ? */
    float T_Nm;             /* 轮扭矩输出 [N·m] */
    float Tp_Nmm;           /* 髋力矩输出 [N·mm] */
} balance_t;

void Balance_Update(balance_t *bal, const vmc_t *vmc, const INS_t *ins,
                    float v_wheel_m_s, float dt);
#endif
