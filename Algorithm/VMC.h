/* ====================================================================
 *  VMC - 虚拟模型控制
 *
 *  FK 局部坐标: 从车辆左侧看，车头在画面左边；+X 朝车头，+Y 朝下。
 *  模块边界与力坐标约定:
 *
 *  对外 (控制层): 极坐标力 (F_axis_N, T_p_Nmm)
 *    F_axis_N = 期望沿腿轴力 [N], 方向 O->B (推轮远离车身)
 *    T_p_Nmm = 期望绕接触点广义力矩 [N*mm], 方向 +leg_axis_angle_rad
 *
 *  对内 (运动学层): 直角坐标力 (F_fk_x_N, F_fk_y_N)
 *    由 VMC_force_polar_to_cart() 在模块内部转换为直角坐标,
 *    仅用于喂给 fivebar_force() (直角坐标雅可比 tau = J^T * F)。
 *    直角坐标只存在于 VMC 内部, 永不越过模块边界。
 *    (F_fk_x_N/F_fk_y_N/F_tangent_N 存于 vmc_t 仅为 Keil watch 调试观测, 语义上仍是内部量)
 * ==================================================================== */

#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "INS_task.h"
#include "forward_solution.h"

#define MASS_HALF_G    70.0f

//VMC参数
#define VMC_KP                 15.0f
#define VMC_KD                 0.2f
#define VMC_F_AXIS_MAX_N       250.0f
#define TORQUE_LIMIT           30.0f

//横滚环参数
#define ROLL_KP                100.0f
#define ROLL_KD                20.0f
#define ROLL_MAX_OUT           30.0f

//防劈叉环参数
#define ANTI_SPLIT_KP          30000.0f
#define ANTI_SPLIT_MAX         10000.0f
#define ANTI_SPLIT_KD          2000.0f

typedef struct
{
    // 输入
    float L_t_set_mm;       			// 目标腿长[mm]
    float F_roll_N;         			// 横滚力差 [N]
    float T_p_Nmm;          			// 期望绕接触点广义力矩 [N·mm]

    // 状态
    float L_t_mm;         				// 腿长 = |OB| [mm]
    float leg_axis_angle_rad;         	// B 极角 = atan2(By, Bx) [rad], 站立 ≈ +π/2
    float theta_rel_body_rad;         	// 腿轴与竖直夹角 = leg_axis_angle_rad - π/2 [rad], 站立 = 0
    float L_t_dot_mm_s;     			// 腿伸缩速率 [mm/s] (虚拟弹簧阻尼项)
    float theta_rel_body_dot_rad_s;   	// 腿轴摆动速率 [rad/s] 对极角求导
    float last_L_t_mm;
    float last_leg_axis_angle_rad;
    uint8_t first_flag;     			// 首周期用当前值初始化 last_*, 防止差分算出 dt 尖峰

    // 力链中间量
    float F_axis_N;         			// 期望沿腿轴力 [N] = Kp·(L_t_set_mm-L_t_mm) + Kd·(0-L_t_dot_mm_s) + MASS_HALF_G/cos(theta_rel_body_rad) + F_roll_N
    float F_tangent_N;      			// 切向力 [N] = T_p_Nmm/L_t_mm (沿 n0)
    float F_fk_x_N, F_fk_y_N;       	// 直角坐标力 [N] (模块内部量, 控制层勿用)

    // 输出[N·mm]
    FKSolve fk_s;           			// 正解(含 A/C/B 点、th1/th2、delta 角、状态码) + 雅可比 J + 奇异标志 + tau_a/tau_b

    float L_t_ddot_mm_s2;   			// 腿长二阶导 (离地检测加速度补偿)
    float FN;               			// 估计地面支持力 (离地检测: FN < 阈值 → 空中降级)
    uint8_t leg_flag;       			// 腿长完成标志
} vmc_t;

void VMC_init(vmc_t *vmc);

void VMC_StateUpdate(vmc_t *vmc, float joint_a_angle_rad, float joint_b_angle_rad, float dt);

void VMC_StateUpdateWithVelocity(vmc_t *vmc, float qa, float qb,
                                  float dqa, float dqb, float dt);
void VMC_ForceCalc(vmc_t *vmc, float joint_a_angle_rad, float joint_b_angle_rad);
void VMC_TorqueAllocate(vmc_t *vmc, float torque_limit_Nm);

// 力坐标变换: 极坐标 (F_axis_N,T_p_Nmm) <-> 直角坐标 (F_fk_x_N,F_fk_y_N), 与 B 同坐标系
void VMC_force_polar_to_cart(const Point2D *B, float F_axis_N, float T_p_Nmm,
                             float *F_fk_x_N, float *F_fk_y_N);
void VMC_force_cart_to_polar(const Point2D *B, float F_fk_x_N, float F_fk_y_N,
                             float *F_axis_N, float *T_p_Nmm);

#endif
