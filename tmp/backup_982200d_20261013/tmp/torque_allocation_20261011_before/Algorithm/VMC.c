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
#define USE_ARM_DSP

//数学加速
#ifdef USE_ARM_DSP
# include "arm_math.h"
static float fsin(float x)  { return arm_sin_f32(x); }
static float fcos(float x)  { return arm_cos_f32(x); }
#else
static float fsin(float x)  { return sinf(x); }
static float fcos(float x)  { return cosf(x); }
#endif

static float fatan2(float y, float x) { return atan2f(y, x); }
static float fsqrt(float x)           { return sqrtf(x); }

vmc_t vmc_left_s;
vmc_t vmc_right_s;

void VMC_init(vmc_t *vmc)
{
    vmc->L_t_set_mm = 212.48f;   /* 站立腿长目标 */
    vmc->first_flag = 1;
    vmc->F_axis_N = 0.0f;
    vmc->T_p_Nmm = 0.0f;
    vmc->fk_s.fk.status = FK_NO_SOLUTION;
}

//正解+计算极坐标
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

/* Use velocity feedback to avoid 0/spike rates when CAN positions repeat. */
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
	
	//将目标力扭转回直角坐标
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

/* --------------------------------------------------------------------
 *  力坐标变换: 极坐标 (F_axis_N, T_p_Nmm) -> 直角坐标 (F_fk_x_N, F_fk_y_N)
 *  e0 = B/|B| (O->B), n0 = (-e0_y, +e0_x) = +leg_axis_angle_rad 方向,
 *  F_tangent_N = T_p_Nmm/L_t_mm 为沿 n0 的切向力。
 *  F_fk_x_N/F_fk_y_N 与 B 同坐标系 (FK 输出系), 可直接喂给 fivebar_force。
 *  单位: B [mm], F [N] -> T_p_Nmm [N*mm]。
 * ------------------------------------------------------------------ */
void VMC_force_polar_to_cart(const Point2D *B, float F_axis_N, float T_p_Nmm,
                             float *F_fk_x_N, float *F_fk_y_N)
{
    float L_t_mm = sqrtf(B->x * B->x + B->y * B->y);
    float inv_L_t_mm = 1.0f / L_t_mm;          /* FK 保证 |OB| >= 40mm, 不会除零 */
    float e0_x = B->x * inv_L_t_mm;
    float e0_y = B->y * inv_L_t_mm;
    float F_tangent_N = T_p_Nmm * inv_L_t_mm;

    *F_fk_x_N = F_axis_N * e0_x - F_tangent_N * e0_y;
    *F_fk_y_N = F_axis_N * e0_y + F_tangent_N * e0_x;
}

/* --------------------------------------------------------------------
 *  逆变换: 直角坐标 (F_fk_x_N, F_fk_y_N) -> 极坐标 (F_axis_N, T_p_Nmm)
 *  F_axis_N = F.e0, T_p_Nmm = L_t_mm * (F.n0)。 (平衡层/调试用)
 * ------------------------------------------------------------------ */
void VMC_force_cart_to_polar(const Point2D *B, float F_fk_x_N, float F_fk_y_N,
                             float *F_axis_N, float *T_p_Nmm)
{
    float L_t_mm = sqrtf(B->x * B->x + B->y * B->y);
    float inv_L_t_mm = 1.0f / L_t_mm;
    float e0_x = B->x * inv_L_t_mm;
    float e0_y = B->y * inv_L_t_mm;
    float F_tangent_N = -F_fk_x_N * e0_y + F_fk_y_N * e0_x;  /* F.n0 */

    *F_axis_N = F_fk_x_N * e0_x + F_fk_y_N * e0_y;
    *T_p_Nmm = F_tangent_N * L_t_mm;
}
