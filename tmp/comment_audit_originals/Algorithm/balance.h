/**
*@file balance.h
*@brief 单腿 LQR 平衡参数、状态与接口声明
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/11
*
*状态顺序为 theta、theta_dot、x、x_dot、phi、phi_dot。
**/
#ifndef __BALANCE_H
#define __BALANCE_H
#include "main.h"
#include "INS_task.h"
#include "VMC.h"

/* 传感器坐标系下的平衡角 [rad]. 这不是质心偏移 y [m]. */
#define THETA_E_RAD         (-0.065042f)
#define PITCH_GYRO_AXIS     0
#define LQR_LEG_SIGN        (1.0f)
#define LQR_PITCH_SIGN      (1.0f)
#define LQR_TP_SIGN         (1.0f)
#define LQR_WHEEL_SIGN      (-1.0f)
#define WHEEL_V_SIGN_L      (-1.0f)
#define WHEEL_V_SIGN_R      (1.0f)
#define WHEEL_RADIUS_M      0.06f
/* 电机转子 RPM -> 输出轮线速度换算系数 (沿用现有值). */
#define WHEEL_RPM_TO_M_S    0.000327f
#define LQR_T_LIMIT_NM      5.0f
#define LQR_TP_LIMIT_NMM    13000.0f
/* 对 v_R-v_L 的负反馈, 以模型前向轮力矩表示. */
#define WHEEL_DIFF_KD       1.0f   /* N*m/(m/s), 阻尼增益 */
#define WHEEL_DIFF_LIMIT_NM 0.8f

typedef struct
{
    float theta_rad, dtheta_rad_s;   /* 本腿角度状态 */
    float x_m, v_m_s;                /* 镜像共享值, watch 用 */
    float phi_rad, dphi_rad_s;       /* 机身姿态 (INS 共享) */
    float T_Nm;                      /* 本腿轮扭矩指令 (驱动系, 已乘 LQR_WHEEL_SIGN) */
    float Tp_Nmm;                    /* 本腿髋力矩指令 */
} balance_t;

typedef struct
{
    float x_m;                       /* 整车共享位移 (单积分器, 左任务独占更新) */
    float v_m_s;                     /* 整车共享速度 (两轮平均) */
    float v_L_m_s, v_R_m_s;          /* 单轮速度 (差速阻尼用) */
    float theta_avg_rad;        	 /* 双腿平均腿角 (对称模态状态, 左腿任务更新) */
    float dtheta_avg_rad_s;      	 /* 双腿平均摆动角速度 */
} balance_shared_t;

float Balance_WheelVelocity(float rpm, float rpm_sign);
void Balance_SharedUpdate(balance_shared_t *sh, float rpm_left, float rpm_right,
                          float dt);
void Balance_Update(balance_t *bal, const balance_shared_t *sh,
                    const vmc_t *vmc, const INS_t *ins);
void Balance_Reset(balance_t *bal);
#endif
