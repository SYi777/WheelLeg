/**
*@file inverse_solution.h
*@brief 平面五连杆机构逆运动学解算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-17
*
*逆解公式 (L1～L4 与坐标均使用 mm, 角度使用 rad)
*
*  已知目标轮心 B=(xb, yb):
*    Rob = sqrt(xb*xb + yb*yb)
*    psi = atan2(yb, xb)
*
*  用三角 OAB、OCB 的余弦定理:
*    phi1 = acos((L1*L1 + Rob*Rob - L3*L3)/(2*L1*Rob))
*    phi2 = acos((L2*L2 + Rob*Rob - L4*L4)/(2*L2*Rob))
*
*  两个输入角各有两个候选:
*    alpha = wrap_pos(psi + phi1) 或 wrap_pos(psi - phi1)
*    beta  = wrap_pos(psi + phi2) 或 wrap_pos(psi - phi2)
*  因此最多有四组 (alpha, beta) 候选。
*
*  候选筛选顺序与 fivebar_ik 实现一致:
*  1) da=wrap(alpha-psi), db=wrap(beta-psi), da、db 同号则淘汰。
*  2) cross_oa_oc=A.x*C.y-A.y*C.x=L1*L2*sin(beta-alpha),
*     按 ASSEMBLY_MODE 所需的叉积符号选择装配解。
*  3) 用 fivebar_fk(alpha,beta) 验证末端坐标:
*     dx=fk.B.x-xb, dy=fk.B.y-yb,
*     dx*dx+dy*dy <= IK_POS_TOL*IK_POS_TOL。
*
*  坐标系: 从车辆左侧看, 车头在画面左边; O=(0,0), +X 朝车头(画面左), +Y 朝下, 角度逆时针为正
**/

#ifndef INVERSE_SOLUTION_H
#define INVERSE_SOLUTION_H

#include <stdbool.h>
#include <stdint.h>
#include "forward_solution.h"

// 逆解状态码
typedef enum {
    IK_OK = 0,
    IK_ERROR_UNREACHABLE,       // B 超出工作空间
    IK_ERROR_TRIANGLE,          // 三角 OAB / OCB 不成立
    IK_ERROR_NO_SOLUTION,       // 所有候选被淘汰
} IKStatus;

// 逆解结果
typedef struct {
    IKStatus status;
    float    alpha;             // 左驱动转角[rad]
    float    beta;              // 右驱动转角[rad]
    Point2D  B;                 // 闭环验证末端点[mm]
    bool     elbow_down;        // true 表示肘下垂
} IKResult;

// 接口

/**
 * @brief  逆运动学核心
 * @param  xb  末端目标 X 坐标[mm]
 * @param  yb  末端目标 Y 坐标[mm]
 * @return IKResult 解算结果 (解选择由 ASSEMBLY_MODE 宏控制)
 */
IKResult fivebar_ik(float xb, float yb);
#endif // INVERSE_SOLUTION_H
