/**
 * @file    inverse_solution.h
 * @brief   平面五连杆机构逆运动学解算
 * @Author  SYi777
 * @Email   jyh1621839418@163.com
 * @Date    2026-6-17
 *
 * 逆解公式
 *
 *   已知 B(xb, yb):
 *
 *     Rob = √(xb2 + yb2)
 *     ψ   = atan2(yb, xb)
 *
 *             l12 + Rob2 ? l32
 *     φ? = ──────────────────              (三角 OAB)
 *               2 · l1 · Rob
 *
 *             l22 + Rob2 ? l42
 *     φ? = ──────────────────              (三角 OCB)
 *               2 · l2 · Rob
 *
 *     α = ψ ± φ?       (＋→ A 在 OB 左侧,  ?→ A 在 OB 右侧)
 *     β = ψ ± φ?       (＋→ C 在 OB 左侧,  ?→ C 在 OB 右侧)
 *
 *     4 种组合:  A左C左 / A左C右 / A右C左 / A右C右
 *
 * ─── 判据 (4 → 2 → 1) ───
 *
 *   ① theta 相对值判同侧
 *      δα = α ? ψ = ±φ?,   δβ = β ? ψ = ±φ?
 *      δα 与 δβ 同号 → A,C 在 OB 同侧 → 淘汰
 *      → 筛掉 A左C左、A右C右, 保留 A左C右、A右C左
 *
 *   ② OA×OC 叉乘判交换
 *      OA×OC = l1·l2·sin(β?α)
 *      交换解 (A左C右 ? A右C左): β?α 反号 → 叉乘反号
 *      装配模式定符号, 异号者淘汰 → 唯一定解
 *
 *   ③ 正解闭环验证
 *      fivebar_fk(α,β).status == FK_OK
 *      |B_fk ? B_target| < EPS
 *
 *   坐标系: O=(0,0), X→右, Y→上, 角度逆时针为正
 */

#ifndef INVERSE_SOLUTION_H
#define INVERSE_SOLUTION_H

#include <stdbool.h>
#include <stdint.h>
#include "forward_solution.h"

/* ====================================================================
 *  逆解状态码
 * ==================================================================== */
typedef enum {
    IK_OK = 0,
    IK_ERROR_UNREACHABLE,       /* B 超出工作空间                   */
    IK_ERROR_TRIANGLE,          /* 三角 OAB / OCB 不成立           */
    IK_ERROR_NO_SOLUTION,       /* 所有候选被淘汰                   */
} IKStatus;

/* ====================================================================
 *  逆解结果
 * ==================================================================== */
typedef struct {
    IKStatus status;
    float    alpha;             /* 左驱动转角 [rad]                */
    float    beta;              /* 右驱动转角 [rad]                */
    Point2D  B;                 /* 闭环验证末端点                   */
    bool     elbow_down;        /* true = 肘下垂                   */
} IKResult;

/* ====================================================================
 *  API
 * ==================================================================== */

/**
 * @brief  逆运动学核心
 * @param  xb  末端目标 X 坐标 [mm]
 * @param  yb  末端目标 Y 坐标 [mm]
 * @return IKResult  (解选择通过 ASSEMBLY_MODE 宏控制)
 */
IKResult fivebar_ik(float xb, float yb);
#endif /* INVERSE_SOLUTION_H */
