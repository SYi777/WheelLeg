/**
*@file forward_solution.h
*@brief 平面五连杆机构正运动学解算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-17
*
*机构拓扑
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
*角度定义 (所有角度以逆时针为正, 参考轴为 +X)
*
*  alpha        : 输入杆 OA 转角 [rad], 从 +X 轴逆时针量至 OA
*  beta         : 输入杆 OC 转角 [rad], 从 +X 轴逆时针量至 OC
*  th1          : 从动杆 AB 全局绝对角 [rad], 从 +X 轴逆时针量至 AB, 范围 [0, 2*pi)
*  th2          : 从动杆 CB 全局绝对角 [rad], 从 +X 轴逆时针量至 CB, 范围 [0, 2*pi)
*  delta_input  : 主动杆 (曲柄 OA 与 OC) 相对夹角 [rad], [0, pi]; 0=两曲柄同向(伸腿极限),
*                 越大曲柄越张开、腿越短; pi=两曲柄反向共线(收腿翻转死点); 用于主动侧限位
*  delta_passive: 从动杆 (AB 与 CB) 相对夹角 [rad], [0, pi]; 0=两杆平行(伸腿极限),
*                 越大腿越短; pi=两杆共线反向(雅可比奇异, sin(th2-th1)=0); 用于从动侧干涉判断
*
*坐标系: 从车辆左侧看, 车头在画面左边; O=(0,0), +X 朝车头(画面左), +Y 朝下
**/

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
#define L1  210.0f            /* 输入杆 OA, 左驱动 [mm] */
#define L2  210.0f            /* 输入杆 OC, 右驱动 [mm] */
#define L3  250.0f            /* 从动杆 AB, 左侧被动杆 [mm] */
#define L4  250.0f            /* 从动杆 CB, 右侧被动杆 [mm] */

/* ====================================================================
 *  限位约束 (rad)
 * ==================================================================== */
#define DELTA_MIN   ( 15.0f * 0.01745329252f)   /* 15° — 避免连杆折叠 */
#define DELTA_MAX   (175.0f * 0.01745329252f)   /* 175° — 避免奇异 */
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
    FK_NO_SOLUTION,         /* 两圆无交点, 连杆无法到达 */
    FK_OUT_OF_RANGE,        /* 主动杆/从动杆夹角超出设定范围 */
    FK_SINGULARITY,         /* A 与 C 重合, 或方程退化 */
} FKStatus;

typedef struct {
    Point2D A;
    Point2D C;
    Point2D B;
    float th1;              /* 从动杆 AB 全局绝对角 [rad], [0, 2*pi) */
    float th2;              /* 从动杆 CB 全局绝对角 [rad], [0, 2*pi) */
    float delta_input;      /* 主动杆相对夹角 [rad], [0, pi]: 0=曲柄并拢(伸腿), pi=反向共线(收腿死点) */
    float delta_passive;    /* 从动杆相对夹角 [rad], [0, pi]: 0=平行(伸腿极限), pi=共线(雅可比奇异) */
    FKStatus status;
} FKResult;

typedef struct {
    float j11, j12, j21, j22;
} Jacobian;

typedef struct {
    FKResult  fk;
    Jacobian  J;
    bool      jac_ok;
    float     tau_a;        /* 左驱动关节力矩 [N*mm] */
    float     tau_b;        /* 右驱动关节力矩 [N*mm] */
} FKSolve;

/* ====================================================================
 *  API
 * ==================================================================== */

/**
 * @brief  正运动学核心
 * @param  alpha  左驱动转角 [rad]
 * @param  beta  右驱动转角 [rad]
 * @return FKResult 解算结果, 含 A、C、B 坐标 [mm]、th1/th2 [rad]、delta 角 [rad] 与状态码
 *
 * 解选择由 ASSEMBLY_MODE 宏控制: 0=肘下垂, 1=肘上抬。
 */
FKResult fivebar_fk(float alpha, float beta);

bool fivebar_jacobian(float alpha, float beta, float th1, float th2, Jacobian *J);

void fivebar_force(const Jacobian *J, float Fx, float Fy, float *tau_a, float *tau_b);

FKSolve fivebar_solve(float alpha, float beta, float Fx, float Fy);

#ifdef __cplusplus
}
#endif
#endif /* FORWARD_SOLUTION_H */
