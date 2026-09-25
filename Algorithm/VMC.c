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
    vmc->L0_set = 220.0f;
    vmc->first_flag = 1;
    vmc->F0 = 0.0f;
    vmc->Tp = 0.0f;
    vmc->fk_s.fk.status = FK_NO_SOLUTION;
}

//正解+计算极坐标
void VMC_StateUpdate(vmc_t *vmc, float alpha_enc, float beta_enc, float dt)
{
	vmc->fk_s.fk = fivebar_fk(alpha_enc, beta_enc);
	if(vmc->fk_s.fk.status != FK_OK )
	{
		vmc->first_flag = 1;
		return;
	}
	
	float bx = vmc->fk_s.fk.B.x;
	float by = vmc->fk_s.fk.B.y;
	
	vmc->L0    = sqrtf(bx*bx + by*by);
	vmc->phi0  = atan2(by, bx);
	vmc->alpha = vmc->phi0 - PI/2;
	
	if(vmc->first_flag)
	{
		vmc->last_L0   = vmc->L0;
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 0;
	}
	
    vmc->d_L0    = (vmc->L0   - vmc->last_L0)   / dt;
    vmc->d_alpha = (vmc->phi0 - vmc->last_phi0) / dt;
    vmc->last_L0   = vmc->L0;
    vmc->last_phi0 = vmc->phi0;
}

void VMC_ForceCalc(vmc_t *vmc, float alpha_enc, float beta_enc, float Tp)
{
	if(vmc->fk_s.fk.status != FK_OK)
	{
		vmc->fk_s.tau_a = vmc->fk_s.tau_b = 0;
		return;
	}
	
	float F_spring = VMC_KP * (vmc->L0_set - vmc->L0) + VMC_KD * (0.0F - vmc->d_L0);
	float F_gravity = 0;//MASS_HALF_G / cosf(vmc->alpha);
	vmc->F0 = F_spring + F_gravity;
	if(vmc->F0 > VMC_F0_MAX)
	{
		vmc->F0 =  VMC_F0_MAX;
	}
	if(vmc->F0 < -VMC_F0_MAX)
	{
		vmc->F0 = -VMC_F0_MAX;
	}
	
	//将目标力扭转回直角坐标
    float inv_L0 = 1.0f / vmc->L0;
    float e0_x = vmc->fk_s.fk.B.x * inv_L0;
    float e0_y = vmc->fk_s.fk.B.y * inv_L0;
	float Ft = Tp * inv_L0;                      // Tp [N·mm] → 切向力 [N]
    float Fx = vmc->F0 * e0_x - Ft * e0_y;       // n0 = (-e0_y, +e0_x)
    float Fy = vmc->F0 * e0_y + Ft * e0_x;
	
	vmc->fk_s.jac_ok = fivebar_jacobian(alpha_enc, beta_enc, vmc->fk_s.fk.th1, vmc->fk_s.fk.th2, &vmc->fk_s.J);
	
	if(vmc->fk_s.jac_ok)
	{
		fivebar_force(&vmc->fk_s.J, Fx, Fy, &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);
	}
	else
	{
		vmc->fk_s.tau_a = vmc->fk_s.tau_b = 0.0f;
	}
}

void vmc_calc_left(vmc_t *vmc, INS_t *ins, float dt)
{
	
}


