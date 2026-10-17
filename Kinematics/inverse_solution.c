/**
*@file inverse_solution.c
*@brief 平面五连杆机构逆运动学解算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-17
*
*先用余弦定理生成候选角, 再经相对侧、装配叉积与正解闭环三重筛选。
**/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "inverse_solution.h"

// 配置项
#define USE_ARM_DSP

/* 逆解专用几何容差[mm]
 * 与正解 EPS (=1e-6, 用于奇异性/退化检测) 职责分离
 * 几何匹配浮点误差约 0.2mm, 取 0.5mm 足够 */
#define IK_POS_TOL  0.5f

#ifdef USE_ARM_DSP
# include "arm_math.h"
// 单精度正弦封装
static float fsin(float x)  { return arm_sin_f32(x); }
// 单精度余弦封装
static float fcos(float x)  { return arm_cos_f32(x); }
// 单精度反余弦封装
static float facos(float x) { return acosf(x); }
#else
// 单精度正弦封装
static float fsin(float x)  { return sinf(x); }
// 单精度余弦封装
static float fcos(float x)  { return cosf(x); }
// 单精度反余弦封装
static float facos(float x) { return acosf(x); }
#endif

// 单精度反正切封装
static float fatan2(float y, float x) { return atan2f(y, x); }
// 单精度平方根封装
static float fsqrt(float x)           { return sqrtf(x); }
// 单精度绝对值封装
static float fabsf_w(float x)         { return fabsf(x); }

// 角度归算工具

// 角度归算到 [-pi, pi]
static inline float wrap(float a) {
    const float two_pi = 6.283185307179586f;
    const float pi     = 3.141592653589793f;
    a = fmodf(a, two_pi);
    if (a >  pi) a -= two_pi;
    if (a < -pi) a += two_pi;
    return a;
}

// 角度归算到 [0, 2*pi)
static inline float wrap_pos(float a) {
    const float two_pi = 6.283185307179586f;
    a = fmodf(a, two_pi);
    return a < 0.0f ? a + two_pi : a;
}

// 余弦定理

// 按余弦定理求三角形夹角, 并检查可达性
static bool tri_angle(float a, float b, float c, float *phi) {
    float den = 2.0f * a * b;
    if (den < 1e-6f) return false;
    float v = (a*a + b*b - c*c) / den;
    if      (v > 1.0f + EPS) return false;
    else if (v > 1.0f) v = 1.0f;
    if      (v < -1.0f - EPS) return false;
    else if (v < -1.0f) v = -1.0f;
    *phi = facos(v);
    return true;
}

/**
 * @brief  由目标轮心位置求串联腿等效传动输入角
 * @param  xb  目标轮心局部 X 坐标[mm]
 * @param  yb  目标轮心局部 Y 坐标[mm]
 * @return IKResult 解算结果, 含输入角、轮心坐标与状态码
 *
 * 用三角 OAB、OCB 的余弦定理生成候选角, 再按相对侧、装配叉积与正解闭环筛选。
 */
IKResult fivebar_ik(float xb, float yb) {
    IKResult ik = {IK_OK, 0.0f, 0.0f, {0.0f, 0.0f}, false};
    const float pi = 3.141592653589793f;

    // 计算轮心极坐标: Rob[mm], psi[rad]
    float Rob = fsqrt(xb*xb + yb*yb);
    if (Rob < EPS) {
        ik.status = IK_ERROR_UNREACHABLE;
        return ik;
    }
    float psi = fatan2(yb, xb);

    // 余弦定理求 phi1、phi2[rad]
    float phi1, phi2;
    if (!tri_angle(L1, Rob, L3, &phi1) ||
        !tri_angle(L2, Rob, L4, &phi2)) {
        ik.status = IK_ERROR_TRIANGLE;
        return ik;
    }

    // 生成驱动角候选解[rad], na、nb 为各角候选数
    float a_left   = wrap_pos(psi + phi1);
    float a_right  = wrap_pos(psi - phi1);
    float b_left   = wrap_pos(psi + phi2);
    float b_right  = wrap_pos(psi - phi2);

    int na = (phi1 > EPS && phi1 < pi - EPS) ? 2 : 1;
    int nb = (phi2 > EPS && phi2 < pi - EPS) ? 2 : 1;

    // 配对候选解并依次筛选
    for (int ia = 0; ia < na; ia++) {
        float alpha = (ia == 0) ? a_left : a_right;

        float da = wrap(alpha - psi);

        float ax = L1 * fcos(alpha);
        float ay = L1 * fsin(alpha);

        // 预筛: |AB| ≈ L3 (容差 0.5mm)
        float ab_len = fsqrt((xb-ax)*(xb-ax) + (yb-ay)*(yb-ay));
        if (fabsf_w(ab_len - L3) > IK_POS_TOL) continue;

        for (int ib = 0; ib < nb; ib++) {
            float beta = (ib == 0) ? b_left : b_right;

            float db = wrap(beta - psi);

            // ─── 判据①: da、db 同号则淘汰 (同侧) ───
            if (da > EPS && db > EPS) continue;
            if (da < -EPS && db < -EPS) continue;

            // 主动杆不重叠: |beta-alpha| >= DELTA_MIN
            float dab = wrap(beta - alpha);
            if (fabsf_w(dab) < DELTA_MIN) continue;

            // 三角 ABC 可达 (容差 0.5mm)
            float cx = L2 * fcos(beta);
            float cy = L2 * fsin(beta);
            float lac = fsqrt((cx-ax)*(cx-ax) + (cy-ay)*(cy-ay));
            float d34 = L3 > L4 ? L3 - L4 : L4 - L3;
            if (lac < d34 - IK_POS_TOL || lac > L3 + L4 + IK_POS_TOL) continue;

            // ─── 判据②: OA×OC 叉乘符号判装配 ───
            float cross_oa_oc = ax * cy - ay * cx;

#if ASSEMBLY_MODE == 0
            if (cross_oa_oc < EPS) continue;
#else
            if (cross_oa_oc > -EPS) continue;
#endif

            // ─── 判据③: 正解闭环验证 ───
            FKResult fk = fivebar_fk(alpha, beta);
            if (fk.status != FK_OK) continue;

            float dx = fk.B.x - xb;
            float dy = fk.B.y - yb;
            if (dx*dx + dy*dy > IK_POS_TOL * IK_POS_TOL) continue;

            // ─── 肘部判断: c_ab 与 c_ao 异号则肘下垂 ───
            float c_ao = (cx-ax)*(-ay) - (cy-ay)*(-ax);
            float c_ab = (cx-ax)*(fk.B.y-ay) - (cy-ay)*(fk.B.x-ax);
            bool  ed   = (c_ab * c_ao < 0.0f);

            ik.status     = IK_OK;
            ik.alpha      = alpha;
            ik.beta       = beta;
            ik.B          = fk.B;
            ik.elbow_down = ed;
            return ik;
        }
    }

    ik.status = IK_ERROR_NO_SOLUTION;
    return ik;
}
