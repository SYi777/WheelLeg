#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "INS_task.h"
#include "forward_solution.h"

#define MASS_HALF_G   98.0f
#define VMC_KP         5.0f
#define VMC_KD         0.1f
#define VMC_F0_MAX   100.0f
#define TORQUE_LIMIT    5.0f

typedef struct
{
    FKSolve fk_s;       // 正解(含 A/C/B 点、th1/th2、delta 角、状态码) + 雅可比 J + 奇异标志 + tau_a/tau_b

    float L0;           // 腿长 = |OB| [mm]
    float phi0;         // B 极角 = atan2(By, Bx) [rad], 站立 ≈ +π/2
    float alpha;        // 腿轴与竖直夹角 = phi0 - π/2 [rad], 站立 = 0
                        // 重力前馈用: 沿腿轴支撑分量 = mg/(2·cos(alpha))
    float d_L0;         // 腿伸缩速率 [mm/s] (虚拟弹簧阻尼项)
    float d_alpha;      // 腿轴摆动速率 [rad/s] (= d_phi0, 同一量只存一份)

    float L0_set;       // 目标腿长 [mm] = 虚拟弹簧自然长度 (遥控/上层给)
    float F0;           // 期望沿腿轴力 [N] = Kp·(L0_set-L0) + Kd·(0-d_L0) + mg/(2·cos(alpha)) ± ΔF_roll
    float Tp;           // 期望绕接触点广义力矩 [N·mm] (LQR输出)

    float last_L0;
    float last_phi0;
    uint8_t first_flag; // 首周期用当前值初始化 last_*, 防止差分算出 dt 尖峰	

    float dd_L0;     	// 腿长二阶导 (离地检测加速度补偿)
    float FN;       	// 估计地面支持力 (离地检测: FN < 阈值 → 空中降级)
    uint8_t leg_flag;	// 腿长完成标志
} vmc_t;

void VMC_init(vmc_t *vmc);

void VMC_StateUpdate(vmc_t *vmc, float alpha_enc, float beta_enc, float dt);

void VMC_ForceCalc(vmc_t *vmc, float alpha_enc, float beta_enc, float Tp);


#endif
