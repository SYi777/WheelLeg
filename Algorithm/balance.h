/**
*@file balance.h
*@brief 对称 LQR 平衡参数、状态与接口声明
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/11
*
*状态顺序为 theta、theta_dot、x、x_dot、phi、phi_dot。
*双任务架构: 共享量由左腿任务集中计算, 右腿任务只读。
**/
#ifndef __BALANCE_H
#define __BALANCE_H
#include "main.h"
#include "INS_task.h"
#include "VMC.h"

// 传感器坐标系下的平衡角 [rad], 不是质心偏移 y [m]
#define THETA_E_RAD         (-0.065042f)
#define PITCH_GYRO_AXIS     0
// 模型系 -> 传感器/驱动系 的符号换算
#define LQR_LEG_SIGN        (1.0f)   // 腿角
#define LQR_PITCH_SIGN      (1.0f)   // 俯仰
#define LQR_TP_SIGN         (1.0f)   // 髋力矩
#define LQR_WHEEL_SIGN      (-1.0f)  // 轮力矩
#define WHEEL_V_SIGN_L      (-1.0f)  // 左轮转速符号
#define WHEEL_V_SIGN_R      (1.0f)   // 右轮转速符号
#define WHEEL_RADIUS_M      0.06f
// 电机转子 RPM -> 轮心线速度换算系数 [m/s/RPM] (沿用现有值)
#define WHEEL_RPM_TO_M_S    0.000327f
// 限幅
#define LQR_T_LIMIT_NM      5.0f     // 轮力矩 [N*m]
#define LQR_TP_LIMIT_NMM    13000.0f // 髋力矩 [N*mm]
// 对 v_R-v_L 的负反馈 (差速阻尼), 以模型前向轮力矩表示
#define WHEEL_DIFF_KD       1.0f     // 阻尼增益 [N*m/(m/s)]
#define WHEEL_DIFF_LIMIT_NM 0.8f     // 差速阻尼限幅 [N*m]

// 单腿状态 (Keil watch 用)
typedef struct
{
    float theta_rad, dtheta_rad_s;   // 世界系腿角 [rad] / 腿角速度 [rad/s]
    float x_m, v_m_s;                // 镜像共享位移 [m] / 速度 [m/s]
    float phi_rad, dphi_rad_s;       // 机身俯仰 [rad] / 角速度 [rad/s]
    float T_Nm;                      // 本腿轮扭矩指令 [N*m] (驱动系, 已乘 LQR_WHEEL_SIGN)
    float Tp_Nmm;                    // 本腿髋力矩指令 [N*mm]
} balance_t;

// 双腿共享状态: 左腿任务独占更新, 右腿任务只读
typedef struct
{
    // 左腿任务更新的共享量
    float x_m;                       // 整车共享位移 [m] (单积分器)
    float v_m_s;                     // 整车共享速度 [m/s] (两轮平均)
    float v_L_m_s, v_R_m_s;          // 左/右轮速度 [m/s] (差速阻尼用)
    float theta_avg_rad;             // 双腿平均腿角 [rad] (世界系, 已减俯仰)
    float dtheta_avg_rad_s;          // 双腿平均腿角速度 [rad/s]
    float Tp_common_Nmm;             // 共享髋力矩 [N*mm] (对称 LQR 输出, 双腿同值)
    float roll_f_N;                  // 共享横滚力差 [N] (左腿 +, 右腿 -)
    float anti_Nmm;                  // 共享防劈叉力矩 [N*mm] (左腿 -, 右腿 +)
    uint8_t ctrl_ok;                 // 控制总闸: 电机安全/两腿FK/遥控/模式 (左腿任务判定)

    // 右腿任务发布的标量 (左腿任务算对称模态用)
    float theta_rel_R_rad;           // 右腿机体系腿角 [rad]
    float theta_rel_R_dot_rad_s;     // 右腿机体系腿角速度 [rad/s]
} balance_shared_t;

float Balance_WheelVelocity(float rpm, float rpm_sign);
void Balance_SharedUpdate(balance_shared_t *sh, float rpm_left, float rpm_right,
                          float dt);
void Balance_Update(balance_t *bal, const balance_shared_t *sh,
                    const vmc_t *vmc, const INS_t *ins);
void Balance_Reset(balance_t *bal);
#endif
