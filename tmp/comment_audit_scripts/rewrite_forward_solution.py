# -*- coding: utf-8 -*-
import io
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def scan(t):
    spans = []
    i = 0
    n = len(t)
    line = 1
    while i < n:
        c = t[i]
        if c == '\n':
            line += 1
            i += 1
            continue
        if c == '"':
            i += 1
            while i < n:
                if t[i] == '\\':
                    i += 2
                    continue
                if t[i] == '"':
                    i += 1
                    break
                if t[i] == '\n':
                    line += 1
                i += 1
            continue
        if c == "'":
            i += 1
            while i < n:
                if t[i] == '\\':
                    i += 2
                    continue
                if t[i] == "'":
                    i += 1
                    break
                i += 1
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '/':
            j = t.find('\n', i)
            if j == -1:
                j = n
            spans.append((i, j, 'line', line, t[i:j]))
            i = j
            continue
        if c == '/' and i + 1 < n and t[i + 1] == '*':
            j = t.find('*/', i + 2)
            j = n if j == -1 else j + 2
            text = t[i:j]
            spans.append((i, j, 'block', line, text))
            line += text.count('\n')
            i = j
            continue
        i += 1
    return spans


def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t


HEADER_COMMON = '''/**
*@file {fname}
*@brief 平面五连杆机构正运动学解算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-17
*
*机构拓扑
*
*             O (固定坐标系原点, 基座)
*            / \\
*          l1   l2      ← 输入杆 (主动)
*          /     \\
*         A       C     ← 主动关节 (A = 左驱动, C = 右驱动)
*          \\     /
*          l3   l4      ← 从动杆 (被动)
*            \\ /
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
'''

HEADER_C_TAIL = '''*
*重要说明
*  1. th1、th2 是全局绝对角, 可落在 [0, 2*pi) 任意象限, 大于 pi 属于正常现象
*  2. 严禁对绝对角直接减 pi, 会导致杆朝向反向、末端坐标完全错误
*  3. 干涉/限位判断必须使用相对夹角, 与连杆整体姿态无关
*  4. 解选择使用叉积判据 (B 相对 AC 连线的位置), 与绝对角大小无关
*  5. 叉积判据天然免疫连杆整体翻转 180° 的影响
**/'''

REPL_H = {
    0: HEADER_COMMON.format(fname='forward_solution.h') + '**/',
    2: '/* 输入杆 OA, 左驱动 [mm] */',
    3: '/* 输入杆 OC, 右驱动 [mm] */',
    4: '/* 从动杆 AB, 左侧被动杆 [mm] */',
    5: '/* 从动杆 CB, 右侧被动杆 [mm] */',
    7: '/* 15° — 避免连杆折叠 */',
    8: '/* 175° — 避免奇异 */',
    11: '/* 两圆无交点, 连杆无法到达 */',
    12: '/* 主动杆/从动杆夹角超出设定范围 */',
    13: '/* A 与 C 重合, 或方程退化 */',
    14: '/* 从动杆 AB 全局绝对角 [rad], [0, 2*pi) */',
    15: '/* 从动杆 CB 全局绝对角 [rad], [0, 2*pi) */',
    16: '/* 主动杆相对夹角 [rad], [0, pi]: 0=曲柄并拢(伸腿), pi=反向共线(收腿死点) */',
    17: '/* 从动杆相对夹角 [rad], [0, pi]: 0=平行(伸腿极限), pi=共线(雅可比奇异) */',
    18: '/* 左驱动关节力矩 [N*mm] */',
    19: '/* 右驱动关节力矩 [N*mm] */',
    21: '''/**
 * @brief  正运动学核心
 * @param  alpha  左驱动转角 [rad]
 * @param  beta  右驱动转角 [rad]
 * @return FKResult 解算结果, 含 A、C、B 坐标 [mm]、th1/th2 [rad]、delta 角 [rad] 与状态码
 *
 * 解选择由 ASSEMBLY_MODE 宏控制: 0=肘下垂, 1=肘上抬。
 */''',
}

REPL_C = {
    0: HEADER_COMMON.format(fname='forward_solution.c') + HEADER_C_TAIL,
    14: '''/**
 * @brief  由两驱动角求轮心 B 的正运动学
 * @param  alpha  第 1 个驱动角 [rad]
 * @param  beta  第 2 个驱动角 [rad]
 * @return FKResult, 含 A、C、B 坐标 [mm]、th1/th2 [rad]、delta 夹角 [rad] 及状态码
 *
 * 令 dx=C.x-A.x、dy=C.y-A.y; 由 B=A+L3*(cos(th1),sin(th1))=C+L4*(cos(th2),sin(th2)) 得
 * L3*L3=(dx+L4*cos(th2))^2+(dy+L4*sin(th2))^2。
 * 设 a=2*L4*dx、b=2*L4*dy、c=L3*L3-L4*L4-dx*dx-dy*dy;
 * 用 t=tan(th2/2) 解 (a+c)*t*t-2*b*t+(c-a)=0, 再由叉积选择装配解。
 */''',
    15: '// 计算主动关节 A、C 坐标 [mm]',
    16: '// dx: A→C 的 x 分量 [mm] = C.x - A.x',
    17: '// dy: A→C 的 y 分量 [mm] = C.y - A.y',
    18: '// 校验 A、C 间距是否在 [|L3-L4|, L3+L4] 内',
    19: '// 计算辅助角方程系数 (半角 t=tan(th2/2))',
    20: '// a = 2*L4*dx',
    21: '// b = 2*L4*dy',
    22: '// c = L3*L3 - L4*L4 - lac2',
    23: '// 用半角公式求 th2 候选解, 存入 t_cand[2]',
    24: '// 二次项退化 (a+c≈0) 时, t=(c-a)/(2*b)',
    25: '// 因此 th2=2*atan2(c-a, 2*b)',
    26: '// 叉积判断候选轮心 B 与原点 O 在 AC 连线的相对侧',
    27: '// cross_B=dx*(B.y-A.y)-dy*(B.x-A.x), cross_O=dx*(0-A.y)-dy*(0-A.x)',
    28: '// 异侧为肘下垂, 同侧为肘上抬',
    29: '// 预计算 cross_O = AC × AO (与候选解无关, 只算一次)',
    30: '// 同侧多候选时的择优分数',
    31: '// 回代求 th1 [rad]',
    32: '// 末端 B 坐标 [mm] (从 A 侧计算)',
    33: '// 双侧一致性校验 (误差 > 1 mm^2 则丢弃)',
    34: '// 叉积: AC × AB',
    35: '// 判断是否为肘下垂',
    36: '// 肘下垂模式: 必须选异侧解',
    37: '// 择优: 选 B.y 更大的 (+Y 向下, y 越大轮心越低)',
    38: '// 肘上抬模式: 必须选同侧解',
    39: '// 择优: 选 B.y 更小的 (上抬更高)',
    40: '// 计算轮心 B 坐标 [mm]',
    41: '// 校验主动杆与从动杆相对夹角 [rad] (参考: 站立 din≈145°/dpa≈107°, 最短腿实测 174.8°/114.1°)',
    42: '// 主动杆上限 178°: 最短腿位形实测 din=174.83°, 留 3.2° 余量; 180° 为翻转死点',
    43: '// 2026.10.12 23:17 jyh review 感觉 passive 这个变量存在和筛选没意义, delta 宏定义原本就不是给下面小腿的',
    44: '''/**
 * @brief  由驱动角速度求轮心速度的雅可比
 * @param  alpha  第 1 个驱动角 [rad]
 * @param  beta  第 2 个驱动角 [rad]
 * @param  th1  从动杆 AB 的全局绝对角 [rad]
 * @param  th2  从动杆 CB 的全局绝对角 [rad]
 * @param  J  输出 2x2 雅可比矩阵 [mm]
 * @return true 可计算; false 分母接近零 (奇异), 此时 J 清零
 *
 * 令 s1=L1*sin(th1-alpha)、s2=L2*sin(th2-beta);
 * J=[sin(th2)*s1, -sin(th1)*s2; -cos(th2)*s1, cos(th1)*s2]/sin(th2-th1)。
 */''',
    45: '// 按 J 的转置将轮心力 [N] 映射为关节力矩 [N*mm]',
    46: '''/**
 * @brief  依次求正解、雅可比和两驱动关节力矩
 * @param  alpha  第 1 个驱动角 [rad]
 * @param  beta  第 2 个驱动角 [rad]
 * @param  Fx  轮心局部 x 方向力 [N]
 * @param  Fy  轮心局部 y 方向力 [N]
 * @return FKSolve, 含正解、雅可比及两关节力矩 [N*mm]
 */''',
}


def process(rel, repl):
    src = os.path.join(ROOT, rel)
    bak = os.path.join(ROOT, 'tmp', 'comment_audit_originals', rel)
    old = io.open(bak, encoding='gbk').read()
    spans = scan(old)
    parts = []
    last = 0
    for k, (a, b, typ, ln, txt) in enumerate(spans):
        if k not in repl:
            continue
        new_comment = repl[k]
        if typ == 'line':
            assert '\n' not in new_comment, 'line comment must be single-line'
            assert new_comment.startswith('//'), 'line comment must start with //'
        else:
            assert new_comment.startswith('/*') and new_comment.endswith('*/')
        parts.append(old[last:a])
        parts.append(new_comment)
        last = b
    parts.append(old[last:])
    new = ''.join(parts)

    assert strip(old) == strip(new), '代码被改动, 禁止写盘! ' + rel

    check = io.open(src, encoding='gbk').read()
    assert check == old, '目标文件与备份不一致, 可能已被其他流程修改: ' + rel

    io.open(src, 'w', encoding='gbk').write(new)
    print('OK', rel, 'comments replaced:', len(repl), 'of', len(spans))


process('Kinematics/forward_solution.h', REPL_H)
process('Kinematics/forward_solution.c', REPL_C)
