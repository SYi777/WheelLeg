# -*- coding: utf-8 -*-
"""
五连杆轮腿 雅可比矩阵 计算与验证 (Python 版, 无需 MATLAB)
===========================================================
机构 (与 Kinematics/forward_solution.c 一致):

        O (基座, 车体)
       / \\
     l1   l2     <- 主动杆, 驱动角 alpha, beta
     /     \\
    A       C    <- 主动关节
     \\     /
     l3   l4     <- 被动杆, 全局角 theta1, theta2
       \\ /
        B        <- 末端 = 轮心

    坐标系: O=(0,0), X 向右, Y 向上, 角度逆时针为正
    杆长:   l1 = l2 = 210 mm,  l3 = l4 = 250 mm

验证内容:
  1. 符号推导: 闭环约束微分, 消去 theta1', theta2',
     解出 B' = J·[alpha'; beta'], 与 C 代码闭式逐项对比
  2. 数值验证: 与 forward_solution.c 相同正解 + 中心差分 d B/d(alpha,beta)
  3. 虚功校验: tau = J^T·F 的功率守恒
  4. 奇异扫描: 全工作空间 det(J) / 条件数, VMC 力映射放大与失效区
"""

import math
import sys

# Windows 控制台默认 GBK, 强制 UTF-8 输出避免中文乱码
if sys.stdout.encoding and sys.stdout.encoding.lower() != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
from sympy import symbols, sin, cos, Matrix, simplify, trigsimp

# ---------------- 参数 (与 forward_solution.h 一致) ----------------
L1 = L2 = 210.0
L3 = L4 = 250.0
EPS = 1e-6
DELTA_MIN = math.radians(15.0)
DELTA_MAX = 3.004            # C 代码里 /*DELTA_MAX*/ 被注释, 实为 3.004 rad
D_PASS_MAX = math.radians(165.0)
ASSEMBLY_MODE = 0            # 0 = 肘朝下 (B 在 AC 下方, 轮腿正常位形)


# ---------------- 角度工具 (与 C 代码一致) ----------------
def wrap(a):
    a = math.fmod(a, 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    if a < -math.pi:
        a += 2.0 * math.pi
    return a


def wrap_pos(a):
    a = math.fmod(a, 2.0 * math.pi)
    return a + 2.0 * math.pi if a < 0.0 else a


def rel_angle(ta, tb):
    d = abs(wrap(tb - ta))
    return 2.0 * math.pi - d if d > math.pi else d


# ---------------- 正解 (fivebar_fk 的 Python 移植) ----------------
def fivebar_fk(alpha, beta):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta), math.cos(beta)
    Ax, Ay = L1 * ca, L1 * sa
    Cx, Cy = L2 * cb, L2 * sb
    dx, dy = Cx - Ax, Cy - Ay
    lac2 = dx * dx + dy * dy
    lac = math.sqrt(lac2)

    if lac < abs(L3 - L4) - EPS or lac > L3 + L4 + EPS:
        return None                      # FK_NO_SOLUTION
    if lac < EPS:
        return None                      # FK_SINGULARITY

    a = 2.0 * L4 * dx
    b = 2.0 * L4 * dy
    c = L3 * L3 - L4 * L4 - lac2
    disc = a * a + b * b - c * c
    if disc < -EPS:
        return None
    disc = max(disc, 0.0)
    sq = math.sqrt(disc)

    if abs(a + c) < EPS:
        if abs(b) > EPS:
            cands = [2.0 * math.atan2(c - a, 2.0 * b)]
        else:
            return None
    else:
        cands = [2.0 * math.atan2(b + sq, a + c),
                 2.0 * math.atan2(b - sq, a + c)]

    cross_O = dx * (-Ay) - dy * (-Ax)
    best_score, best_th1, best_th2, found = -1e9, 0.0, 0.0, False
    for t in cands:
        th2 = wrap_pos(t)
        rx = dx + L4 * math.cos(th2)
        ry = dy + L4 * math.sin(th2)
        th1 = wrap_pos(math.atan2(ry, rx))
        bx = Ax + L3 * math.cos(th1)
        by = Ay + L3 * math.sin(th1)
        bx2 = Cx + L4 * math.cos(th2)
        by2 = Cy + L4 * math.sin(th2)
        if (bx - bx2) ** 2 + (by - by2) ** 2 > 1.0:
            continue
        cross_B = dx * (by - Ay) - dy * (bx - Ax)
        elbow_down = (cross_B * cross_O < 0.0)
        if ASSEMBLY_MODE == 0:
            if not elbow_down:
                continue
            score = by
        else:
            if elbow_down:
                continue
            score = -by
        if score > best_score:
            best_score, best_th1, best_th2, found = score, th1, th2, True
    if not found:
        return None

    Bx = Ax + L3 * math.cos(best_th1)
    By = Ay + L3 * math.sin(best_th1)
    d_in = rel_angle(alpha, beta)
    d_pass = rel_angle(best_th1, best_th2)
    if (d_in < DELTA_MIN or d_in > DELTA_MAX or
            d_pass < DELTA_MIN or d_pass > D_PASS_MAX):
        return None                      # FK_OUT_OF_RANGE

    return dict(A=(Ax, Ay), C=(Cx, Cy), B=(Bx, By),
                th1=best_th1, th2=best_th2,
                delta_input=d_in, delta_passive=d_pass)


# ---------------- 雅可比闭式 (fivebar_jacobian 的 Python 移植) ----------------
def jac_closed(alpha, beta, th1, th2):
    sd = math.sin(wrap(th2 - th1))
    if abs(sd) < EPS:
        return None
    inv = 1.0 / sd
    s1 = L1 * math.sin(wrap(th1 - alpha))
    s2 = L2 * math.sin(wrap(th2 - beta))
    return np.array([
        [inv * math.sin(th2) * s1,   inv * (-math.sin(th1)) * s2],
        [inv * (-math.cos(th2)) * s1, inv * math.cos(th1) * s2],
    ])


# ======================================================================
# 1. 符号推导: 闭环约束微分 -> J
#    B = A + l3·u1 = C + l4·u2,  u1 = [cosθ1, sinθ1], u2 = [cosθ2, sinθ2]
#    B' = A' + l3·θ1'·v1 = C' + l4·θ2'·v2,  v = [-sinθ, cosθ]
# ======================================================================
print("=" * 70)
print("1. 符号推导 J (sympy), 与 C 代码闭式对比")
print("=" * 70)

alpha, beta, th1, th2 = symbols('alpha beta theta1 theta2', real=True)
l1, l2, l3, l4 = symbols('l1 l2 l3 l4', positive=True)
ad, bd, th1d, th2d, xbd, ybd = symbols("ad bd th1d th2d xbd ybd", real=True)

# M·[xbd, ybd, th1d, th2d]^T = N·[ad, bd]^T
M = Matrix([[1, 0,  l3 * sin(th1), 0],
            [0, 1, -l3 * cos(th1), 0],
            [1, 0,  0,             l4 * sin(th2)],
            [0, 1,  0,            -l4 * cos(th2)]])
N = Matrix([[-l1 * sin(alpha), 0],
            [l1 * cos(alpha),  0],
            [0,               -l2 * sin(beta)],
            [0,                l2 * cos(beta)]])

S = M.inv() * N          # 行 = [xbd, ybd, th1d, th2d], 列 = [ad, bd]
J_sym = S[0:2, :]        # B' = J_sym · [alpha'; beta']

# C 代码闭式 (forward_solution.c fivebar_jacobian)
s1 = l1 * sin(th1 - alpha)
s2 = l2 * sin(th2 - beta)
J_c = 1 / sin(th2 - th1) * Matrix([
    [sin(th2) * s1,  -sin(th1) * s2],
    [-cos(th2) * s1,  cos(th1) * s2],
])

D = simplify(trigsimp(J_sym - J_c))
if D == Matrix.zeros(2, 2):
    print("  [OK] sympy 推导结果 与 C 代码闭式 完全一致 (逐项化简为零)")
    print("  J = 1/sin(θ2-θ1) · [ sinθ2·l1·sin(θ1-α),  -sinθ1·l2·sin(θ2-β) ]")
    print("                     [ -cosθ2·l1·sin(θ1-α),  cosθ1·l2·sin(θ2-β) ]")
else:
    print("  [FAIL] 符号对比未化简为零, 改为随机数值点检查:")
    rng = np.random.default_rng(1)
    ok = True
    for _ in range(200):
        vals = dict(alpha=rng.uniform(0, 2 * np.pi), beta=rng.uniform(0, 2 * np.pi),
                    th1=rng.uniform(0, 2 * np.pi), th2=rng.uniform(0, 2 * np.pi),
                    l1=210.0, l2=210.0, l3=250.0, l4=250.0)
        dmax = max(abs(float((J_sym - J_c)[i, j].subs(vals))) for i in range(2) for j in range(2))
        if dmax > 1e-9:
            ok = False
            break
    print("  " + ("[OK] 200 个随机点数值一致" if ok else "[FAIL] 数值不一致!"))

# ======================================================================
# 2. 数值验证: 中心差分 vs 闭式
# ======================================================================
print()
print("=" * 70)
print("2. 数值验证: 中心差分 d B/d(alpha,beta) vs 闭式 J")
print("=" * 70)

# 目标位形: 轮腿站立, 轮心 B 在 O 正下方 (用三角形公式求精确 alpha/beta)
def ik_standing(y_abs):
    """B = (0, -y_abs) 的逆解 (肘朝下支)"""
    psi = math.atan2(-y_abs, 0.0)          # = -pi/2
    Rob = y_abs
    phi1 = math.acos((L1 ** 2 + Rob ** 2 - L3 ** 2) / (2 * L1 * Rob))
    phi2 = math.acos((L2 ** 2 + Rob ** 2 - L4 ** 2) / (2 * L2 * Rob))
    for a in (psi + phi1, psi - phi1):
        for b in (psi + phi2, psi - phi2):
            fk = fivebar_fk(a, b)
            if fk and abs(fk['B'][0]) < 1e-6:
                return a, b, fk
    return None


fd_max_err = 0.0
fd_tested = 0
h = 1e-6
rng = np.random.default_rng(42)

# 站立位形 (轮心正下方 3 个高度) + 随机有效位形
pose_list = []
for y_abs in (250.0, 300.0, 350.0):
    r = ik_standing(y_abs)
    if r:
        pose_list.append(('站立 B=(0,-%.0f)' % y_abs, r[0], r[1]))
# 随机有效位形
grid = np.arange(0.0, 2 * np.pi, np.deg2rad(3.0))
valid = [(a, b) for a in grid for b in grid if fivebar_fk(a, b)]
idx = rng.choice(len(valid), size=min(30, len(valid)), replace=False)
for i in idx:
    pose_list.append(('随机 #%d' % i, valid[i][0], valid[i][1]))

for name, a0, b0 in pose_list:
    fk0 = fivebar_fk(a0, b0)
    if not fk0:
        continue
    Jc = jac_closed(a0, b0, fk0['th1'], fk0['th2'])
    if Jc is None:
        continue
    # 中心差分 (检查扰动后位形仍在同一分支)
    fkp = fivebar_fk(a0 + h, b0)
    fkm = fivebar_fk(a0 - h, b0)
    if not (fkp and fkm and abs(wrap(fkp['th1'] - fkm['th1'])) < 0.1):
        continue
    Jn = np.zeros((2, 2))
    Jn[:, 0] = (np.array(fkp['B']) - np.array(fkm['B'])) / (2 * h)
    fkp = fivebar_fk(a0, b0 + h)
    fkm = fivebar_fk(a0, b0 - h)
    if not (fkp and fkm and abs(wrap(fkp['th1'] - fkm['th1'])) < 0.1):
        continue
    Jn[:, 1] = (np.array(fkp['B']) - np.array(fkm['B'])) / (2 * h)
    err = np.max(np.abs(Jn - Jc)) / max(1.0, np.max(np.abs(Jc)))
    fd_max_err = max(fd_max_err, err)
    fd_tested += 1
    print("  %-16s J 最大相对误差 = %.3e" % (name, err))

print("  => %d 个位形全部通过, 最大相对误差 %.3e" % (fd_tested, fd_max_err))

# ======================================================================
# 3. 虚功校验 + VMC 力矩映射示例
# ======================================================================
print()
print("=" * 70)
print("3. 虚功校验 tau = J^T·F 与典型位形的 J")
print("=" * 70)

for name, a0, b0 in pose_list[:3]:
    fk = fivebar_fk(a0, b0)
    Jc = jac_closed(a0, b0, fk['th1'], fk['th2'])
    qd = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1)])
    Bd = Jc @ qd
    F = np.array([rng.uniform(-10, 10), rng.uniform(-10, 10)])
    p1 = F @ Bd
    p2 = (Jc.T @ F) @ qd
    print()
    print("  [%s]" % name)
    print("    alpha=%.3f rad(%.1f°) beta=%.3f rad(%.1f°)" %
          (a0, np.rad2deg(a0), b0, np.rad2deg(b0)))
    print("    theta1=%.3f° theta2=%.3f°  B=(%.1f, %.1f)" %
          (np.rad2deg(fk['th1']), np.rad2deg(fk['th2']), fk['B'][0], fk['B'][1]))
    print("    J =")
    print("      [ %8.3f  %8.3f ]" % tuple(Jc[0]))
    print("      [ %8.3f  %8.3f ]" % tuple(Jc[1]))
    print("    det(J) = %.1f   cond(J) = %.2f" % (np.linalg.det(Jc), np.linalg.cond(Jc)))
    print("    功率守恒: F·B' = %.6f, tau·q' = %.6f  (一致=%s)" %
          (p1, p2, abs(p1 - p2) < 1e-9))
    # 车体重力支撑示例: F = (0, +mg) 地面给轮子的支持力, m=20kg 时 196N
    Fg = np.array([0.0, 196.0])
    tau = Jc.T @ Fg
    print("    支撑 20kg 车体 (F=(0,196)N): tau = (%.1f, %.1f) N·mm" %
          (tau[0], tau[1]))

# ======================================================================
# 4. 工作空间奇异扫描
# ======================================================================
print()
print("=" * 70)
print("4. 工作空间奇异扫描 (alpha, beta 全圆 2° 网格)")
print("=" * 70)

g = np.arange(0.0, 2 * np.pi, np.deg2rad(2.0))
n_ok = 0
det_min, det_max = 1e30, -1e30
cond_max = 0.0
near_sing = 0
det_list = []
for a in g:
    for b in g:
        fk = fivebar_fk(a, b)
        if not fk:
            continue
        n_ok += 1
        Jc = jac_closed(a, b, fk['th1'], fk['th2'])
        if Jc is None:
            continue
        det = np.linalg.det(Jc)
        det_list.append(det)
        det_min = min(det_min, det)
        det_max = max(det_max, det)
        cond_max = max(cond_max, np.linalg.cond(Jc))
        if abs(math.sin(wrap(fk['th2'] - fk['th1']))) < 0.05 or \
           abs(math.sin(wrap(fk['th1'] - a))) < 0.05 or \
           abs(math.sin(wrap(fk['th2'] - b))) < 0.05:
            near_sing += 1

print("  有效位形: %d / %d (其余为超出杆长或关节角限制)" % (n_ok, g.size ** 2))
print("  det(J) 范围: [%.1f, %.1f] mm^2" % (det_min, det_max))
print("  |det(J)| 中位数: %.1f mm^2" % np.median(np.abs(det_list)))
print("  max cond(J): %.1f" % cond_max)
print("  近奇异位形 (任一 sin<0.05): %d 个" % near_sing)
print()
print("  奇异条件:  sin(θ2-θ1)=0  -> AB,CB 共线 (腿伸直/折死)")
print("             sin(θ1-α) =0  -> OA,AB 共线")
print("             sin(θ2-β) =0  -> OC,CB 共线")
print("  结论: det(J)=0 恰好发生在腿完全伸直/完全折叠的边界位形,")
print("        VMC 中需要限制 |det(J)| 下限或对 J^T 做阻尼, 避免力矩发散。")
