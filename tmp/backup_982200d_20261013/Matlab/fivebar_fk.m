function fk = fivebar_fk(alpha, beta)
% FIVEBAR_FK  平面五连杆轮腿正运动学 (与 Kinematics/forward_solution.c 完全一致)
%
% 机构拓扑 (见 forward_solution.c 开头注释):
%        O (固定坐标系原点, 基座, 两主动杆公共铰点)
%       / \
%     l1   l2      主动杆, 驱动角 alpha, beta (A=左驱动, C=右驱动)
%     /     \
%    A       C     主动关节
%     \     /
%     l3   l4      从动杆 (被动), 全局角 theta1, theta2
%       \ /
%        B         末端执行点 = 轮心
%
% 输入:
%   alpha, beta [rad]  主动杆 OA/OC 转角 (逆时针为正, 参考 +X)
% 输出:
%   fk 结构体:
%     .A, .C, .B   2x1 列向量, 各关节点坐标 [mm]
%     .th1, .th2   从动杆全局绝对角 [rad, 0~2pi)
%     .delta_input, .delta_passive  相对夹角 [rad, 0~pi]
%     .status      0=OK  1=NO_SOLUTION  2=OUT_OF_RANGE  3=SINGULARITY
%
% 注意: 主动侧限位 DELTA_MAX 与 C 代码一致, 取 3.004 rad (~172.1 度)

L1 = 210; L2 = 210; L3 = 250; L4 = 250;
EPS = 1e-6;

fk.A = [0; 0]; fk.C = [0; 0]; fk.B = [0; 0];
fk.th1 = 0; fk.th2 = 0;
fk.delta_input = 0; fk.delta_passive = 0;
fk.status = 1;                    % 默认 NO_SOLUTION

% ---- Step 1: 主动关节坐标 ----
sa = sin(alpha); ca = cos(alpha);
sb = sin(beta);  cb = cos(beta);
fk.A = [L1*ca; L1*sa];
fk.C = [L2*cb; L2*sb];

dx = fk.C(1) - fk.A(1);
dy = fk.C(2) - fk.A(2);
lac2 = dx*dx + dy*dy;
lac = sqrt(lac2);

% ---- Step 2: 几何可达性 ----
if lac < abs(L3-L4) - EPS || lac > L3+L4 + EPS
    return;
end
if lac < EPS
    fk.status = 3;                % FK_SINGULARITY
    return;
end

% ---- Step 3: 辅助角方程系数  a·cos(th2) + b·sin(th2) = c ----
a = 2*L4*dx;
b = 2*L4*dy;
c = L3*L3 - L4*L4 - lac2;

disc = a*a + b*b - c*c;
if disc < -EPS
    return;
end
disc = max(disc, 0);
sq = sqrt(disc);

% ---- Step 4: 万能公式求 theta2 候选 ----
if abs(a + c) < EPS
    % 退化: 一次方程
    if abs(b) > EPS
        tc = 2*atan2(c - a, 2*b);
    else
        return;
    end
else
    tc = [2*atan2(b + sq, a + c), 2*atan2(b - sq, a + c)];
end

% ---- Step 5: 叉积判据选解 (肘下垂 ASSEMBLY_MODE=0) ----
cross_O = dx * (-fk.A(2)) - dy * (-fk.A(1));

best_score = -1e9;
best_th1 = 0; best_th2 = 0;
found = false;

for t = tc
    th2 = wrap_pos(t);
    rx = dx + L4*cos(th2);
    ry = dy + L4*sin(th2);
    th1 = wrap_pos(atan2(ry, rx));

    bx = fk.A(1) + L3*cos(th1);
    by = fk.A(2) + L3*sin(th1);

    % 双链一致性校验
    bx2 = fk.C(1) + L4*cos(th2);
    by2 = fk.C(2) + L4*sin(th2);
    if (bx-bx2)^2 + (by-by2)^2 > 1.0
        continue;
    end

    % 判据: B 与 O 在 AC 异侧 => 肘下垂
    cross_B = dx*(by - fk.A(2)) - dy*(bx - fk.A(1));
    elbow_down = (cross_B * cross_O < 0);

    if ~elbow_down
        continue;                 % ASSEMBLY_MODE = 0 只选肘下垂
    end
    score = by;                   % tiebreak: 取 B.y 更大 (更低)
    if score > best_score
        best_score = score;
        best_th1 = th1; best_th2 = th2;
        found = true;
    end
end

if ~found
    return;
end

fk.th1 = best_th1;
fk.th2 = best_th2;
fk.B = fk.A + L3 * [cos(fk.th1); sin(fk.th1)];

% ---- Step 6: 关节限位校验 ----
fk.delta_input   = rel_angle(alpha, beta);
fk.delta_passive = rel_angle(fk.th1, fk.th2);

if fk.delta_input   < deg2rad(15) || fk.delta_input   > 3.004 || ...
   fk.delta_passive < deg2rad(15) || fk.delta_passive > deg2rad(165)
    fk.status = 2;                % FK_OUT_OF_RANGE
    return;
end

fk.status = 0;                    % FK_OK
end

% ==================== 角度工具 (与 C 代码一致) ====================
function a = wrap(a)
% 归算到 [-pi, pi)
a = mod(a, 2*pi);
if a > pi
    a = a - 2*pi;
end
end

function a = wrap_pos(a)
% 归算到 [0, 2pi)
a = mod(a, 2*pi);
end

function d = rel_angle(ta, tb)
% 两角相对夹角, 归算到 [0, pi]
d = abs(wrap(tb - ta));
if d > pi
    d = 2*pi - d;
end
end
