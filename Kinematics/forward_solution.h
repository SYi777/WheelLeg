/**
 * @file    forward_solution.h
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
 */

#ifndef FORWARD_SOLUTION_H
#define FORWARD_SOLUTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 *  杆长参数 (mm)
 * ==================================================================== */
#define L1  210.0f            /* 输入杆 OA — 左驱动        */
#define L2  210.0f            /* 输入杆 OC — 右驱动        */
#define L3  250.0f            /* 从动杆 AB — 左侧被动杆    */
#define L4  250.0f            /* 从动杆 CB — 右侧被动杆    */

/* ====================================================================
 *  限位约束 (rad)
 * ==================================================================== */
#define DELTA_MIN   ( 15.0f * 0.01745329252f)   /*  15° — 避免连杆折叠 */
#define DELTA_MAX   (165.0f * 0.01745329252f)   /* 165° — 避免奇异   */
#define EPS          1e-6f

/* ====================================================================
 *  装配模式 (编译期宏)
 *  0 = 肘下垂 (B 与 O 在 AC 异侧, 末端向下自然垂落)
 *  1 = 肘上抬 (B 与 O 在 AC 同侧, 末端向上收起)
 * ==================================================================== */
#define ASSEMBLY_MODE  0

/* ====================================================================
 *  类型定义
 * ==================================================================== */
typedef struct { float x, y; } Point2D;

typedef enum {
    FK_OK = 0,
    FK_NO_SOLUTION,         /* 两圆无交点, 连杆无法到达              */
    FK_OUT_OF_RANGE,        /* 主动杆/从动杆夹角超出 [Δmin, Δmax]   */
    FK_SINGULARITY,         /* A 与 C 重合 或 方程退化              */
} FKStatus;

typedef struct {
    Point2D A;
    Point2D C;
    Point2D B;
    float th1;              /* 从动杆 AB 全局绝对角 [0, 2π)        */
    float th2;              /* 从动杆 CB 全局绝对角 [0, 2π)        */
    float delta_input;      /* 主动杆相对夹角 [0, π]               */
    float delta_passive;    /* 从动杆相对夹角 [0, π] — 干涉判断用   */
    FKStatus status;
} FKResult;

typedef struct {
    float j11, j12, j21, j22;
} Jacobian;

typedef struct {
    FKResult  fk;
    Jacobian  J;
    bool      jac_ok;
    float     tau_a;        /* 左驱动关节力矩 */
    float     tau_b;        /* 右驱动关节力矩 */
} FKSolve;

/* ====================================================================
 *  API
 * ==================================================================== */

/**
 * @brief  正运动学核心
 * @param  alpha  左驱动转角 [rad]
 * @param  beta   右驱动转角 [rad]
 * @return FKResult  (解选择通过 ASSEMBLY_MODE 宏控制)
 */
FKResult fivebar_fk(float alpha, float beta);

bool fivebar_jacobian(float alpha, float beta, float th1, float th2, Jacobian *J);

void fivebar_force(const Jacobian *J, float Fx, float Fy, float *tau_a, float *tau_b);

FKSolve fivebar_solve(float alpha, float beta, float Fx, float Fy);

#ifdef __cplusplus
}
#endif
#endif /* FORWARD_SOLUTION_H */
