/**
 * @file    forward_solution.c
 * @brief   平面五连杆机构正运动学解算
 * @Author  SYi777
 * @Email   jyh1621839418@163.com
 * @Date    2026-6-17
 *
 * 机构拓扑
 *
 *             O (固定坐标系原点, 基座)
 *            / \
 *          l1   l2      ← 输入杆 (主动)
 *          /     \
 *         A       C     ← 主动关节 (A = 左驱动, C = 右驱动)
 *          \     /
 *          l3   l4      ← 从动杆 (被动)
 *            \ /
 *             B         ← 末端执行点
 *
 * ─── 角度定义 (所有角度以逆时针为正, 参考轴为 +X) ───
 *
 *   α  (alpha) : 输入杆 OA 的转角, 从 +X 轴逆时针量至 OA
 *   β  (beta)  : 输入杆 OC 的转角, 从 +X 轴逆时针量至 OC
 *   θ1 (theta1): 从动杆 AB 的全局绝对角, 从 +X 轴逆时针量至 AB, 范围 [0, 2π)
 *   θ2 (theta2): 从动杆 CB 的全局绝对角, 从 +X 轴逆时针量至 CB, 范围 [0, 2π)
 *   Δθ_input   : 主动杆相对夹角, 归算到 [0, π], 用于主动侧限位
 *   Δθ_passive : 从动杆相对夹角, 归算到 [0, π], 用于从动侧干涉判断
 *
 *   坐标系: O = (0,0),  X 轴向右为正,  Y 轴向上为正
 *
 * ─── 重要说明 ───
 *   1. θ?、θ? 是全局绝对角，可落在 0~2π 任意象限，大于 π 属于正常现象
 *   2. 严禁对绝对角直接减 π，会导致杆朝向反向、末端坐标完全错误
 *   3. 干涉/限位判断必须使用「相对夹角」，与连杆整体姿态无关
 *   4. 解选择使用「叉积判据」(B 相对 AC 连线的位置)，与绝对角大小无关
 *   5. 叉积判据天然免疫连杆整体翻转 180° 的影响
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "forward_solution.h"

/* ===================== 配置项 ===================== */
/* 是否使用 ARM DSP 库加速三角函数 */
#define USE_ARM_DSP

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

/* ===================== 角度归算工具 ===================== */

/* 归一到 [-π, π) */
static inline float wrap(float a) {
    const float two_pi = 6.283185307179586f;
    const float pi     = 3.141592653589793f;
    a = fmodf(a, two_pi);
    if (a >  pi) a -= two_pi;
    if (a < -pi) a += two_pi;
    return a;
}

/* 归一到 [0, 2π) */
static inline float wrap_pos(float a) {
    const float two_pi = 6.283185307179586f;
    a = fmodf(a, two_pi);
    return a < 0.0f ? a + two_pi : a;
}

/* 计算两杆的相对夹角，归算到 [0, π]，用于限位/干涉判断 */
static inline float relative_angle(float th_a, float th_b) {
    float d = fabsf(wrap(th_b - th_a));
    const float pi = 3.141592653589793f;
    return d > pi ? 2.0f * pi - d : d;
}

/* 主动杆夹角：归算到 [0, π] */
static inline float delta_input(float alpha, float beta) {
    return relative_angle(alpha, beta);
}

/* ====================================================================
 *  fivebar_fk — 正运动学核心
 *
 *  输入: alpha (α) — 左驱动转角 [rad]
 *        beta  (β) — 右驱动转角 [rad]
 *
 *  推导路径:
 *    1) A = l1·[cosα, sinα],  C = l2·[cosβ, sinβ]
 *    2) 联立: B = A + l3·[cosθ1, sinθ1] = C + l4·[cosθ2, sinθ2]
 *    3) 移项平方消 θ1:
 *       l32 = |C ? A + l4·[cosθ2, sinθ2]|2
 *       l32 = lAC2 + l42 + 2·l4·(xC?xA)·cosθ2 + 2·l4·(yC?yA)·sinθ2
 *    4) 整理为标准辅助角方程  a·cosθ2 + b·sinθ2 = c
 *       a = 2·l4·(xC ? xA)
 *       b = 2·l4·(yC ? yA)
 *       c = l32 ? l42 ? lAC2
 *    5) 万能公式 t = tan(θ2/2): (a+c)·t2 ? 2b·t + (c?a) = 0
 *       解得 θ2 = 2·atan2(b ± √(a2+b2?c2), a+c)
 *    6) 回代得 θ1 = atan2(dy + l4·sinθ2, dx + l4·cosθ2)
 *    7) 按叉积判据选解 (B 相对 AC 连线的位置)，区分肘下垂/肘上抬
 *    8) B = A + l3·[cosθ1, sinθ1]
 *
 *  异常处理:
 *    - lac < EPS       → FK_SINGULARITY  (A 与 C 重合)
 *    - lac 超出可达区间 → FK_NO_SOLUTION  (三角形两边之和/差约束)
 *    - Δθ 超出限位     → FK_OUT_OF_RANGE (主动杆/从动杆夹角越界)
 * ==================================================================== */
FKResult fivebar_fk(float alpha, float beta) {
    FKResult r = {0};

    /* ---- Step 1: 主动关节坐标 ---- */
    float sa = fsin(alpha), ca = fcos(alpha);
    float sb = fsin(beta),  cb = fcos(beta);

    r.A.x = L1 * ca;  r.A.y = L1 * sa;
    r.C.x = L2 * cb;  r.C.y = L2 * sb;

    float dx   = r.C.x - r.A.x;    /* xC ? xA */
    float dy   = r.C.y - r.A.y;    /* yC ? yA */
    float lac2 = dx*dx + dy*dy;
    float lac  = fsqrt(lac2);

    /* ---- Step 2: 几何可达性校验 ---- */
    float sum = L3 + L4;
    float dif = L3 > L4 ? L3 - L4 : L4 - L3;

    if (lac < dif - EPS || lac > sum + EPS) {
        r.status = FK_NO_SOLUTION;
        return r;
    }
    if (lac < EPS) {
        r.status = FK_SINGULARITY;
        return r;
    }

    /* ---- Step 3: 辅助角方程系数 ---- */
    float a = 2.0f * L4 * dx;             /* 2·l4·(xC ? xA) */
    float b = 2.0f * L4 * dy;             /* 2·l4·(yC ? yA) */
    float c = L3*L3 - L4*L4 - lac2;       /* l32 ? l42 ? lAC2 */

    float disc = a*a + b*b - c*c;
    if (disc < -EPS) {
        r.status = FK_NO_SOLUTION;
        return r;
    }
    if (disc < 0.0f) disc = 0.0f;
    float sqrt_d = fsqrt(disc);

    /* ---- Step 4: 万能公式求两候选 θ2 ---- */
    float t_cand[2];
    int   nc = 0;

    if (fabsf(a + c) < EPS) {
        /* 二次项退化: (a+c)·t2 ? 2b·t + (c?a) = 0
           退化为 ?2b·t + (c?a) = 0 → t = (c?a)/(2b)
           即 θ2 = 2·atan2(c?a, 2b)                              */
        if (fabsf(b) > EPS) {
            t_cand[nc++] = 2.0f * fatan2(c - a, 2.0f * b);
        } else {
            r.status = FK_NO_SOLUTION;
            return r;
        }
    } else {
        t_cand[nc++] = 2.0f * fatan2(b + sqrt_d, a + c);
        t_cand[nc++] = 2.0f * fatan2(b - sqrt_d, a + c);
    }

    /* ---- Step 5: 叉积判据选解 ----
     *
     *  核心思想: 判断候选 B 在 AC 连线的哪一侧。
     *
     *  叉积 cross = AC × AB = dx·(B.y?A.y) ? dy·(B.x?A.x)
     *  叉积 cross_O = AC × AO = dx·( 0?A.y) ? dy·( 0?A.x)
     *
     *  - cross 和 cross_O 异号 → B 与原点 O 在 AC 异侧 → 肘下垂
     *  - cross 和 cross_O 同号 → B 与原点 O 在 AC 同侧 → 肘上抬
     *
     *  此判据:
     *    ? 与 θ?、θ? 的绝对大小无关
     *    ? 与连杆是否翻转 180° 无关
     *    ? 与 A、C 的左右顺序无关
     */

    /* 预计算 cross_O = AC × AO (与候选解无关, 只算一次) */
    float cross_O = dx * (-r.A.y) - dy * (-r.A.x);

    float best_score = -1e9f;   /* 用于同侧多候选时的 tiebreak */
    float best_th2   = 0.0f;
    float best_th1   = 0.0f;
    int   found      = 0;

    for (int i = 0; i < nc; i++) {
        float th2 = wrap_pos(t_cand[i]);

        /* 回代求 θ1 */
        float rx = dx + L4 * fcos(th2);
        float ry = dy + L4 * fsin(th2);
        float th1 = wrap_pos(fatan2(ry, rx));

        /* 末端坐标 (从 A 侧计算) */
        float bx = r.A.x + L3 * fcos(th1);
        float by = r.A.y + L3 * fsin(th1);

        /* 双侧一致性校验 */
        float bx2 = r.C.x + L4 * fcos(th2);
        float by2 = r.C.y + L4 * fsin(th2);
        float err = (bx - bx2)*(bx - bx2) + (by - by2)*(by - by2);
        if (err > 1.0f) continue;

        /* 叉积: AC × AB */
        float cross_B = dx * (by - r.A.y) - dy * (bx - r.A.x);

        /* 判断是否为肘下垂 */
        bool is_elbow_down = (cross_B * cross_O < 0.0f);

#if ASSEMBLY_MODE == 0
        /* 肘下垂模式: 必须选异侧解 */
        if (!is_elbow_down) continue;
        /* tiebreak: 选 B.y 更大的 (下垂更深, Y 向上时 y 更小 = 更下) */
        float score = by;
#else
        /* 肘上抬模式: 必须选同侧解 */
        if (is_elbow_down) continue;
        /* tiebreak: 选 B.y 更小的 (上抬更高) */
        float score = -by;
#endif
        if (score > best_score) {
            best_score = score;
            best_th2   = th2;
            best_th1   = th1;
            found      = 1;
        }
    }

    if (!found) {
        r.status = FK_NO_SOLUTION;
        return r;
    }

    r.th1 = best_th1;
    r.th2 = best_th2;

    /* ---- Step 6: 末端坐标 ---- */
    r.B.x = r.A.x + L3 * fcos(r.th1);
    r.B.y = r.A.y + L3 * fsin(r.th1);

    /* ---- Step 7: 夹角限位校验 ---- */
    r.delta_input   = delta_input(alpha, beta);
    r.delta_passive = relative_angle(r.th1, r.th2);

    if (r.delta_input   < DELTA_MIN || r.delta_input   > 3.004/*DELTA_MAX*/ ||
        r.delta_passive < DELTA_MIN || r.delta_passive > DELTA_MAX) {
        r.status = FK_OUT_OF_RANGE;
        return r;
    }

    r.status = FK_OK;
    return r;
}

/* ====================================================================
 *  fivebar_jacobian — 雅可比矩阵
 *
 *  B' = J · [α', β']^T
 *
 *        1           [  sinθ2·s1   ?sinθ1·s2 ]
 *  J = ───── · [                            ]
 *      sinΔθ       [ ?cosθ2·s1    cosθ1·s2 ]
 *
 *  s1 = l1·sin(θ1?α),   s2 = l2·sin(θ2?β),   Δθ = θ2?θ1
 *
 *  返回 false → 奇异 (sinΔθ≈0, 两被动杆平行, det(J)=0)
 * ==================================================================== */
bool fivebar_jacobian(float alpha, float beta, float th1, float th2, Jacobian *J)
{
    float sd = fsin(wrap(th2 - th1));
    if (fabsf(sd) < EPS)
	{
        J->j11 = J->j12 = J->j21 = J->j22 = 0.0f;
        return false;
    }

    float inv_sd = 1.0f / sd;
    float s1 = L1 * fsin(wrap(th1 - alpha));
    float s2 = L2 * fsin(wrap(th2 - beta));

    float st2 = fsin(th2), ct2 = fcos(th2);
    float st1 = fsin(th1), ct1 = fcos(th1);

    J->j11 =  inv_sd *  st2 * s1;
    J->j12 =  inv_sd * -st1 * s2;
    J->j21 =  inv_sd * -ct2 * s1;
    J->j22 =  inv_sd *  ct1 * s2;

    return true;
}

/* ====================================================================
 *  fivebar_force — 力控逆解
 *
 *  τ = J^T · F
 *  τ[0] = τα = j11·Fx + j21·Fy   (左驱动关节力矩)
 *  τ[1] = τβ = j12·Fx + j22·Fy   (右驱动关节力矩)
 * ==================================================================== */
void fivebar_force(const Jacobian *J, float Fx, float Fy, float *tau_a, float *tau_b)
{
    *tau_a = J->j11 * Fx + J->j21 * Fy;
    *tau_b = J->j12 * Fx + J->j22 * Fy;
}

/* ====================================================================
 *  fivebar_solve — 一站式求解: 正解 → 雅可比 → 力矩
 * ==================================================================== */
FKSolve fivebar_solve(float alpha, float beta, float Fx, float Fy)
{
    FKSolve s = {0};
    s.fk = fivebar_fk(alpha, beta);

    if (s.fk.status == FK_OK)
	{
        s.jac_ok = fivebar_jacobian(alpha, beta, s.fk.th1, s.fk.th2, &s.J);
        if (s.jac_ok)
		{
            fivebar_force(&s.J, Fx, Fy, &s.tau_a, &s.tau_b);
        }
    }
    return s;
}
