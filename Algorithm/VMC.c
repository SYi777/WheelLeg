/**
*@file VMC.c
*@brief 虚拟模型控制与关节力矩映射
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026/10/10
*
*通过等效运动学的雅可比，将腿轴力和髋力矩映射到关节。
**/
/* ====================================================================
 *  VMC - 虚拟模型控制
 *  力坐标约定见 VMC.h 头部: 对外极坐标 (F_axis_N, T_p_Nmm),
 *  直角坐标 (F_fk_x_N, F_fk_y_N) 只在本模块内部使用 (喂给 fivebar_force)。
 *  关节力矩经 VMC_TorqueAllocate() 按 TORQUE_LIMIT 限幅分配后下发。
 * ==================================================================== */

#include "VMC.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>

vmc_t vmc_left_s;
vmc_t vmc_right_s;

/**
 * @brief  初始化本腿 VMC 状态与站立腿长目标
 * @param  vmc  本腿 VMC 状态
 * @return 无
 *
 * 清零力与力矩指令, 正解状态置为 FK_NO_SOLUTION。
 */
void VMC_init(vmc_t *vmc)
{
    vmc->L_t_set_mm = 212.48f;   // 站立腿长目标
    vmc->first_flag = 1;
    vmc->F_axis_N = 0.0f;
    vmc->T_p_Nmm = 0.0f;
    vmc->fk_s.fk.status = FK_NO_SOLUTION;
	
	vmc->F_axis_applied_N = vmc->T_p_applied_Nmm = 0.0f;   // 分配后指令清零
    vmc->axis_force_limited = vmc->hip_moment_limited = 0U; // 限幅标志清零
}

/**
 * @brief  由等效正解计算轮心位置、腿长与腿轴角速度
 * @param  vmc  本腿 VMC 状态
 * @param  joint_a_angle_rad  第 1 个驱动角 [rad]
 * @param  joint_b_angle_rad  第 2 个驱动角 [rad]
 * @param  dt  本次控制周期 [s]
 * @return 无
 *
 * 局部坐标 +x 朝车头、+y 朝下；角速度由相邻周期位置差计算。
 * FK 失败时置 first_flag, 恢复后用当前值重置差分基准, 避免 dt 尖峰。
 */
void VMC_StateUpdate(vmc_t *vmc, float joint_a_angle_rad, float joint_b_angle_rad, float dt)
{
	vmc->fk_s.fk = fivebar_fk(joint_a_angle_rad, joint_b_angle_rad);
	if(vmc->fk_s.fk.status != FK_OK )
	{
		vmc->first_flag = 1;
		return;
	}
	
	float bx = vmc->fk_s.fk.B.x;
	float by = vmc->fk_s.fk.B.y;
	
	vmc->L_t_mm    = sqrtf(bx*bx + by*by);
	// FK: +x 朝车头，+y 朝下；B=(0,+L_t) 时腿竖直向下。
	vmc->leg_axis_angle_rad  = atan2(by, bx);
	vmc->theta_rel_body_rad = vmc->leg_axis_angle_rad - PI/2;
	
	if(vmc->first_flag)
	{
		vmc->last_L_t_mm   = vmc->L_t_mm;
        vmc->last_leg_axis_angle_rad = vmc->leg_axis_angle_rad;
        vmc->first_flag = 0;
	}
	
    vmc->L_t_dot_mm_s    = (vmc->L_t_mm   - vmc->last_L_t_mm)   / dt;
    vmc->theta_rel_body_dot_rad_s = (vmc->leg_axis_angle_rad - vmc->last_leg_axis_angle_rad) / dt;
    vmc->last_L_t_mm   = vmc->L_t_mm;
    vmc->last_leg_axis_angle_rad = vmc->leg_axis_angle_rad;
}

/**
 * @brief  用关节反馈速度和雅可比更新轮心速度
 * @param  vmc  本腿 VMC 状态
 * @param  qa  第 1 个驱动角 [rad]
 * @param  qb  第 2 个驱动角 [rad]
 * @param  dqa  第 1 个驱动角速度 [rad/s]
 * @param  dqb  第 2 个驱动角速度 [rad/s]
 * @param  dt  本次控制周期 [s]
 * @return 无
 *
 * 用雅可比 J 把关节速度映射为轮心速度 (vx,vy), 再投影到腿轴方向与切向,
 * 得到 L_t_dot 与极角速度, 精度优于位置差分。
 */
void VMC_StateUpdateWithVelocity(vmc_t *vmc, float qa, float qb,
                                  float dqa, float dqb, float dt)
{
    VMC_StateUpdate(vmc, qa, qb, dt);
    if (vmc->fk_s.fk.status != FK_OK) return;
    vmc->fk_s.jac_ok = fivebar_jacobian(qa, qb, vmc->fk_s.fk.th1,
                                      vmc->fk_s.fk.th2, &vmc->fk_s.J);
    if (!vmc->fk_s.jac_ok) return;
    const Jacobian *j = &vmc->fk_s.J;
    const FKResult *fk = &vmc->fk_s.fk;
    float vx = j->j11 * dqa + j->j12 * dqb;
    float vy = j->j21 * dqa + j->j22 * dqb;
    float length2 = fk->B.x * fk->B.x + fk->B.y * fk->B.y;
    vmc->L_t_dot_mm_s = (fk->B.x * vx + fk->B.y * vy) / vmc->L_t_mm;
    vmc->theta_rel_body_dot_rad_s = (fk->B.x * vy - fk->B.y * vx) / length2;
}

/**
 * @brief  计算腿轴力并映射为两驱动关节力矩
 * @param  vmc  本腿 VMC 状态及目标髋力矩
 * @param  joint_a_angle_rad  第 1 个驱动角 [rad]
 * @param  joint_b_angle_rad  第 2 个驱动角 [rad]
 * @return 无
 *
 * 腿轴力 = 长度弹簧阻尼 + 重力补偿 + 横滚修正:
 *   F_spring  = VMC_KP*(L_t_set_mm-L_t_mm) + VMC_KD*(0-L_t_dot_mm_s)  [N]
 *   F_gravity = MASS_HALF_G/cos(theta_rel_body_rad)                    [N]
 * F_spring_N/F_gravity_N 保存截断前分量 (调试观测), F_axis_N 夹到
 * ±VMC_F_AXIS_MAX_N 后转直角坐标求 tau, 再交 VMC_TorqueAllocate() 分配。
 */
void VMC_ForceCalc(vmc_t *vmc, float joint_a_angle_rad, float joint_b_angle_rad)
{
	if(vmc->fk_s.fk.status != FK_OK)
	{
		vmc->fk_s.tau_a = vmc->fk_s.tau_b = 0;
		return;
	}
	
	float F_spring = VMC_KP * (vmc->L_t_set_mm - vmc->L_t_mm) + VMC_KD * (0.0F - vmc->L_t_dot_mm_s);
	float F_gravity = MASS_HALF_G / cosf(vmc->theta_rel_body_rad);
	vmc->F_spring_N = F_spring;
    vmc->F_gravity_N = F_gravity;
    vmc->F_axis_N = F_spring + F_gravity + vmc->F_roll_N;
	if(vmc->F_axis_N > VMC_F_AXIS_MAX_N)
	{
		vmc->F_axis_N =  VMC_F_AXIS_MAX_N;
	}
	if(vmc->F_axis_N < -VMC_F_AXIS_MAX_N)
	{
		vmc->F_axis_N = -VMC_F_AXIS_MAX_N;
	}
	
	// 目标极坐标力 -> 局部直角坐标力
	vmc->F_tangent_N = vmc->T_p_Nmm / vmc->L_t_mm;
	VMC_force_polar_to_cart(&vmc->fk_s.fk.B, vmc->F_axis_N, vmc->T_p_Nmm, &vmc->F_fk_x_N, &vmc->F_fk_y_N);
	
	vmc->fk_s.jac_ok = fivebar_jacobian(joint_a_angle_rad, joint_b_angle_rad, vmc->fk_s.fk.th1, vmc->fk_s.fk.th2, &vmc->fk_s.J);
	
	if(vmc->fk_s.jac_ok)
	{
		fivebar_force(&vmc->fk_s.J, vmc->F_fk_x_N, vmc->F_fk_y_N, &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);
		VMC_TorqueAllocate(vmc, TORQUE_LIMIT);// 按 TORQUE_LIMIT 限幅分配
	}
	else
	{
		vmc->fk_s.tau_a = vmc->fk_s.tau_b = 0.0f;
	}
}


/**
 * @brief  将腿轴力和髋力矩转换到正解局部直角坐标
 * @param  B  局部坐标中的轮心位置 [mm]
 * @param  F_axis_N  沿 O 到 B 的腿轴力 [N]
 * @param  T_p_Nmm  绕腿轴角正向的广义力矩 [N*mm]
 * @param  F_fk_x_N  输出局部 x 方向力 [N]
 * @param  F_fk_y_N  输出局部 y 方向力 [N]
 * @return 无
 *
 * 切向单位向量 n0 = (-B.y, B.x)/|B|,
 * F_cart = F_axis*e0 + F_tangent*n0, 其中 e0 = B/|B|, F_tangent = T_p/|B|。
 */
void VMC_force_polar_to_cart(const Point2D *B, float F_axis_N, float T_p_Nmm,
                             float *F_fk_x_N, float *F_fk_y_N)
{
    float L_t_mm = sqrtf(B->x * B->x + B->y * B->y);
    float inv_L_t_mm = 1.0f / L_t_mm;          // FK 保证 |OB| >= 40mm, 不会除零
    float e0_x = B->x * inv_L_t_mm;
    float e0_y = B->y * inv_L_t_mm;
    float F_tangent_N = T_p_Nmm * inv_L_t_mm;

    *F_fk_x_N = F_axis_N * e0_x - F_tangent_N * e0_y;
    *F_fk_y_N = F_axis_N * e0_y + F_tangent_N * e0_x;
}

/**
 * @brief  将局部直角坐标力转换为腿轴力和髋力矩
 * @param  B  局部坐标中的轮心位置 [mm]
 * @param  F_fk_x_N  局部 x 方向力 [N]
 * @param  F_fk_y_N  局部 y 方向力 [N]
 * @param  F_axis_N  输出腿轴力 [N]
 * @param  T_p_Nmm  输出广义髋力矩 [N*mm]
 * @return 无
 *
 * 投影: F_axis = F·e0, F_tangent = F·n0, T_p = F_tangent*|B|,
 * 其中 e0 = B/|B|, n0 = (-B.y, B.x)/|B|。
 */
void VMC_force_cart_to_polar(const Point2D *B, float F_fk_x_N, float F_fk_y_N,
                             float *F_axis_N, float *T_p_Nmm)
{
    float L_t_mm = sqrtf(B->x * B->x + B->y * B->y);
    float inv_L_t_mm = 1.0f / L_t_mm;
    float e0_x = B->x * inv_L_t_mm;
    float e0_y = B->y * inv_L_t_mm;
    float F_tangent_N = -F_fk_x_N * e0_y + F_fk_y_N * e0_x;  // 切向分量 F·n0 [N]

    *F_axis_N = F_fk_x_N * e0_x + F_fk_y_N * e0_y;
    *T_p_Nmm = F_tangent_N * L_t_mm;
}

/**
 * @brief  按关节力矩上限分配腿轴力与髋力矩, 并重算关节力矩
 * @param  vmc  本腿 VMC 状态 (输入 F_axis_N/T_p_Nmm, 输出分配结果)
 * @param  torque_limit_Nm  单关节力矩上限 [N*m]
 * @return 无
 *
 * 四步:
 * 1) 记录截断前力矩: tau_request_a_Nm/tau_request_b_Nm = 当前 tau/1000;
 * 2) 髋力矩优先: 把 T_p 产生的两关节力矩等比压进 ±torque_limit_Nm,
 *    超限时整体缩小 moment_Nm, 并置 hip_moment_limited = 1;
 * 3) 用剩余扭矩空间夹 F_axis: 由 tau_i = force_gain[i]*F + hip_torque[i]
 *    反解 |tau_i| <= torque_limit_Nm 的可行区间并夹入, 被夹时置
 *    axis_force_limited = 1, 区间为空则取 F = 0;
 * 4) 用可行力重算 J^T F: 由 F_axis_applied_N/T_p_applied_Nmm 转直角坐标,
 *    调用 fivebar_force() 写回 tau_a/tau_b。
 *
 * 记 e0 = B/|B|, n0 = (-e0.y, e0.x), 则 tau_i = force_gain[i]*F_axis + hip_torque[i]:
 *   force_gain[i]  = (J^T e0)_i / 1000      轴向力到关节力矩的增益 [N*m/N]
 *   moment_gain[i] = (J^T n0)_i / |B|       髋力矩到关节力矩的增益 (无量纲)
 *   hip_torque[i]  = moment_gain[i]*T_p     髋力矩分量产生的关节力矩 [N*m]
 * 分配后指令存 F_axis_applied_N/T_p_applied_Nmm, 供 Keil watch 调试观测。
 */
void VMC_TorqueAllocate(vmc_t *vmc, float torque_limit_Nm)
{
    vmc->tau_request_a_Nm = vmc->fk_s.tau_a / 1000.0f;// 分配前关节力矩 [N*m]
    vmc->tau_request_b_Nm = vmc->fk_s.tau_b / 1000.0f;// 分配前关节力矩 [N*m]
    vmc->axis_force_limited = vmc->hip_moment_limited = 0U;// 清限幅标志
    if (vmc->fk_s.fk.status != FK_OK || !vmc->fk_s.jac_ok
        || vmc->L_t_mm <= 0.0f || torque_limit_Nm <= 0.0f)
    {
        vmc->F_axis_applied_N = vmc->T_p_applied_Nmm = 0.0f;// 无效状态清零
        vmc->fk_s.tau_a = vmc->fk_s.tau_b = 0.0f;
        return;
    }
    const Jacobian *j = &vmc->fk_s.J;
    float ex = vmc->fk_s.fk.B.x / vmc->L_t_mm;// 腿轴单位向量 e0
    float ey = vmc->fk_s.fk.B.y / vmc->L_t_mm;// 腿轴单位向量 e0
    float force_gain[2] = {(j->j11 * ex + j->j21 * ey) / 1000.0f,
                           (j->j12 * ex + j->j22 * ey) / 1000.0f};
    float moment_gain[2] = {(-j->j11 * ey + j->j21 * ex) / vmc->L_t_mm,
                            (-j->j12 * ey + j->j22 * ex) / vmc->L_t_mm};
    float moment_Nm = vmc->T_p_Nmm / 1000.0f;// 期望髋力矩 [N*m]
    float hip_torque[2] = {moment_gain[0] * moment_Nm,
                           moment_gain[1] * moment_Nm};
    float peak = fmaxf(fabsf(hip_torque[0]), fabsf(hip_torque[1]));// 两关节峰值 [N*m]
    if (peak > torque_limit_Nm)
    {
        float scale = torque_limit_Nm / peak;// 等比缩小系数
        moment_Nm *= scale;
        hip_torque[0] *= scale;
        hip_torque[1] *= scale;
        vmc->hip_moment_limited = 1U;
    }
    float force_low = -FLT_MAX, force_high = FLT_MAX;// 可行轴向力区间 [N]
    for (int i = 0; i < 2; ++i)
    {
        if (fabsf(force_gain[i]) < 1e-8f) continue;
        float a = (-torque_limit_Nm - hip_torque[i]) / force_gain[i];// tau=-limit 对应力 [N]
        float b = ( torque_limit_Nm - hip_torque[i]) / force_gain[i];// tau=+limit 对应力 [N]
        float low = fminf(a, b), high = fmaxf(a, b);
        force_low = fmaxf(force_low, low);
        force_high = fminf(force_high, high);
    }
    float force_N = vmc->F_axis_N;// 夹入可行区间
    if (force_N < force_low) force_N = force_low;
    if (force_N > force_high) force_N = force_high;
    // 髋力矩等比缩放后, 零轴向力总是可行
    if (force_low > force_high) force_N = 0.0f;
    vmc->axis_force_limited = fabsf(force_N - vmc->F_axis_N) > 1e-3f;// 记录是否被夹
    vmc->F_axis_applied_N = force_N;// 分配后可行指令
    vmc->T_p_applied_Nmm = moment_Nm * 1000.0f;// 分配后髋力矩 [N*mm]
    vmc->F_tangent_N = vmc->T_p_applied_Nmm / vmc->L_t_mm;// 实际切向力 [N]
    VMC_force_polar_to_cart(&vmc->fk_s.fk.B, force_N, vmc->T_p_applied_Nmm,
                            &vmc->F_fk_x_N, &vmc->F_fk_y_N);
    fivebar_force(j, vmc->F_fk_x_N, vmc->F_fk_y_N,
                   &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);// 用可行力重算关节力矩 [N*mm]
}

