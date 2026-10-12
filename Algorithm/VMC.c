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
 *  VMC - Virtual Model Control (虚拟模型控制)
 *  力坐标约定见 VMC.h 头部: 对外极坐标 (F_axis_N, T_p_Nmm), 直角坐标 (F_fk_x_N, F_fk_y_N)
 *  只在本模块内部使用 (喂给直角坐标雅可比 fivebar_force), 不出边界。
 *  Force convention: see VMC.h - polar (F_axis_N, T_p_Nmm) at the boundary;
 *  Cartesian (F_fk_x_N, F_fk_y_N) is internal only (feeds fivebar_force).
 * ==================================================================== */

#include "VMC.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <float.h>

vmc_t vmc_left_s;
vmc_t vmc_right_s;

// 初始化本腿 VMC 状态和站立腿长目标
void VMC_init(vmc_t *vmc)
{
    vmc->L_t_set_mm = 212.48f;   // 站立腿长目标
    vmc->first_flag = 1;
    vmc->F_axis_N = 0.0f;
    vmc->T_p_Nmm = 0.0f;
    vmc->fk_s.fk.status = FK_NO_SOLUTION;
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
 * 腿轴力包含长度弹簧阻尼、重力补偿和横滚修正。
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
    vmc->F_axis_N = F_spring + F_gravity + vmc->F_roll_N;
	if(vmc->F_axis_N > VMC_F_AXIS_MAX_N)
	{
		vmc->F_axis_N =  VMC_F_AXIS_MAX_N;
	}
	if(vmc->F_axis_N < -VMC_F_AXIS_MAX_N)
	{
		vmc->F_axis_N = -VMC_F_AXIS_MAX_N;
	}
	
	// 将目标极坐标力转换为局部直角坐标力
	vmc->F_tangent_N = vmc->T_p_Nmm / vmc->L_t_mm;
	VMC_force_polar_to_cart(&vmc->fk_s.fk.B, vmc->F_axis_N, vmc->T_p_Nmm, &vmc->F_fk_x_N, &vmc->F_fk_y_N);
	
	vmc->fk_s.jac_ok = fivebar_jacobian(joint_a_angle_rad, joint_b_angle_rad, vmc->fk_s.fk.th1, vmc->fk_s.fk.th2, &vmc->fk_s.J);
	
	if(vmc->fk_s.jac_ok)
	{
		fivebar_force(&vmc->fk_s.J, vmc->F_fk_x_N, vmc->F_fk_y_N, &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);
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
 * 切向单位向量为 (-B.y, B.x)/|B|。
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
 */
void VMC_force_cart_to_polar(const Point2D *B, float F_fk_x_N, float F_fk_y_N,
                             float *F_axis_N, float *T_p_Nmm)
{
    float L_t_mm = sqrtf(B->x * B->x + B->y * B->y);
    float inv_L_t_mm = 1.0f / L_t_mm;
    float e0_x = B->x * inv_L_t_mm;
    float e0_y = B->y * inv_L_t_mm;
    float F_tangent_N = -F_fk_x_N * e0_y + F_fk_y_N * e0_x;  // F.n0

    *F_axis_N = F_fk_x_N * e0_x + F_fk_y_N * e0_y;
    *T_p_Nmm = F_tangent_N * L_t_mm;
}
