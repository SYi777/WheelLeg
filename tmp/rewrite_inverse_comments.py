# -*- coding: utf-8 -*-
"""按用户注释风格重写 inverse_solution.h/.c, 仅改注释, 写盘前校验代码不变。"""
import io
import os
import re
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')


def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)   # 块注释
    t = re.sub(r'//[^\n]*', '', t)                  # 行注释
    return t


H_HEADER = """/**
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
**/"""

H_DOC = """/**
 * @brief  逆运动学核心
 * @param  xb  末端目标 X 坐标[mm]
 * @param  yb  末端目标 Y 坐标[mm]
 * @return IKResult 解算结果 (解选择由 ASSEMBLY_MODE 宏控制)
 */"""

C_HEADER = """/**
*@file inverse_solution.c
*@brief 平面五连杆机构逆运动学解算
*@Author SYi777
*@Email jyh1621839418@163.com
*@Date 2026-6-17
*
*先用余弦定理生成候选角, 再经相对侧、装配叉积与正解闭环三重筛选。
**/"""

C_TOL = """/* 逆解专用几何容差[mm]
 * 与正解 EPS (=1e-6, 用于奇异性/退化检测) 职责分离
 * 几何匹配浮点误差约 0.2mm, 取 0.5mm 足够 */"""

C_DOC = """/**
 * @brief  由目标轮心位置求串联腿等效传动输入角
 * @param  xb  目标轮心局部 X 坐标[mm]
 * @param  yb  目标轮心局部 Y 坐标[mm]
 * @return IKResult 解算结果, 含输入角、轮心坐标与状态码
 *
 * 用三角 OAB、OCB 的余弦定理生成候选角, 再按相对侧、装配叉积与正解闭环筛选。
 */"""


def build_h_reps(lines):
    def trailing(idx, text):
        return lines[idx].split('/*')[0] + text

    return [
        (0, 32, H_HEADER.splitlines()),
        (40, 43, ['// 逆解状态码']),
        (45, 48, [
            trailing(45, '// B 超出工作空间'),
            trailing(46, '// 三角 OAB / OCB 不成立'),
            trailing(47, '// 所有候选被淘汰'),
        ]),
        (50, 53, ['// 逆解结果']),
        (55, 59, [
            trailing(55, '// 左驱动转角[rad]'),
            trailing(56, '// 右驱动转角[rad]'),
            trailing(57, '// 闭环验证末端点[mm]'),
            trailing(58, '// true 表示肘下垂'),
        ]),
        (61, 64, ['// 接口']),
        (65, 71, H_DOC.splitlines()),
        (72, 73, ['#endif // INVERSE_SOLUTION_H']),
    ]


def build_c_reps(lines):
    return [
        (0, 7, C_HEADER.splitlines()),
        (13, 14, ['// 配置项']),
        (16, 19, C_TOL.splitlines()),
        (23, 24, ['// 单精度正弦封装']),
        (25, 26, ['// 单精度余弦封装']),
        (27, 28, ['// 单精度反余弦封装']),
        (30, 31, ['// 单精度正弦封装']),
        (32, 33, ['// 单精度余弦封装']),
        (34, 35, ['// 单精度反余弦封装']),
        (38, 39, ['// 单精度反正切封装']),
        (40, 41, ['// 单精度平方根封装']),
        (42, 43, ['// 单精度绝对值封装']),
        (45, 46, ['// 角度归算工具']),
        (47, 48, ['// 角度归算到 [-pi, pi]']),
        (57, 58, ['// 角度归算到 [0, 2*pi)']),
        (64, 65, ['// 余弦定理']),
        (66, 67, ['// 按余弦定理求三角形夹角, 并检查可达性']),
        (79, 87, C_DOC.splitlines()),
        (91, 92, ['    // 计算轮心极坐标: Rob[mm], psi[rad]']),
        (99, 100, ['    // 余弦定理求 phi1、phi2[rad]']),
        (107, 108, ['    // 生成驱动角候选解[rad], na、nb 为各角候选数']),
        (116, 117, ['    // 配对候选解并依次筛选']),
        (125, 126, ['        // 预筛: |AB| ≈ L3 (容差 0.5mm)']),
        (134, 135, ['            // ─── 判据①: da、db 同号则淘汰 (同侧) ───']),
        (138, 139, ['            // 主动杆不重叠: |beta-alpha| >= DELTA_MIN']),
        (142, 143, ['            // 三角 ABC 可达 (容差 0.5mm)']),
        (149, 150, ['            // ─── 判据②: OA×OC 叉乘符号判装配 ───']),
        (158, 159, ['            // ─── 判据③: 正解闭环验证 ───']),
        (166, 167, ['            // ─── 肘部判断: c_ab 与 c_ao 异号则肘下垂 ───']),
    ]


def process(rel_path, builder):
    backup = os.path.join(BASE, 'tmp/comment_audit_originals/' + rel_path)
    target = os.path.join(BASE, rel_path)
    old_text = io.open(backup, encoding='gbk').read()
    old_lines = old_text.splitlines()
    reps = builder(old_lines)
    new_lines = list(old_lines)
    for start, end, repl in sorted(reps, key=lambda r: r[0], reverse=True):
        new_lines[start:end] = repl
    new_text = '\n'.join(new_lines) + '\n'

    if strip(old_text) != strip(new_text):
        print('FAIL:', rel_path, '代码被改动, 禁止写盘!')
        import difflib
        old_s = strip(old_text)
        new_s = strip(new_text)
        for d in difflib.unified_diff(old_s.splitlines(), new_s.splitlines(),
                                      'old', 'new', lineterm=''):
            print(d)
        return False

    raw = io.open(backup, 'rb').read()
    eol = '\r\n' if b'\r\n' in raw else '\n'
    io.open(target, 'w', encoding='gbk', newline='').write(
        new_text.replace('\n', eol))
    print('OK:', rel_path, 'lines', len(old_lines), '->', len(new_lines),
          'eol=', repr(eol))
    return True


if __name__ == '__main__':
    ok = True
    ok &= process('Kinematics/inverse_solution.h', build_h_reps)
    ok &= process('Kinematics/inverse_solution.c', build_c_reps)
    sys.exit(0 if ok else 1)
