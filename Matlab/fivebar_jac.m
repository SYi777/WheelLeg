function [J, ok] = fivebar_jac(alpha, beta, th1, th2)
% FIVEBAR_JAC  五连杆轮腿雅可比矩阵 (与 Kinematics/forward_solution.c 的
%              fivebar_jacobian() 闭式完全一致)
%
% 定义:  B' = J · [alpha'; beta']^T
%
%              1        [  sin(th2)·s1   -sin(th1)·s2 ]
%  J = ----------------- [                             ]
%      sin(th2 - th1)    [ -cos(th2)·s1    cos(th1)·s2 ]
%
% 其中:
%   s1 = L1·sin(th1 - alpha)
%   s2 = L2·sin(th2 - beta)
%
% 输入:
%   alpha, beta [rad]  主动杆转角
%   th1, th2    [rad]  从动杆全局绝对角 (由 fivebar_fk 给出)
% 输出:
%   J   2x2 雅可比 [mm/rad]
%   ok  false = 奇异 (两被动杆共线, sin(th2-th1)≈0, det(J)=0)
%
% 力映射 (VMC 用):  tau = J' * F,  F 为作用于轮心 B 的期望力 [N]
%   tau(1) = 左驱动 α 关节力矩,  tau(2) = 右驱动 β 关节力矩  [N·mm]

L1 = 210; L2 = 210;
EPS = 1e-6;

J = zeros(2);
ok = false;

sd = sin(wrap(th2 - th1));
if abs(sd) < EPS
    return;
end

inv_sd = 1 / sd;
s1 = L1 * sin(wrap(th1 - alpha));
s2 = L2 * sin(wrap(th2 - beta));

J = inv_sd * [ sin(th2)*s1,  -sin(th1)*s2;
              -cos(th2)*s1,   cos(th1)*s2 ];
ok = true;
end

function a = wrap(a)
% 归算到 [-pi, pi)
a = mod(a, 2*pi);
if a > pi
    a = a - 2*pi;
end
end
