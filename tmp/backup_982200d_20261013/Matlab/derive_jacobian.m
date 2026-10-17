%% derive_jacobian.m
% 五连杆轮腿 雅可比矩阵 符号推导 (需要 Symbolic Math Toolbox)
%
% 推导思路:
%   末端 B 有两条表达链:  B = A + l3·u1 = C + l4·u2
%     u1 = [cos(th1); sin(th1)],  u2 = [cos(th2); sin(th2)]
%   对时间微分:
%     B' = A' + l3·th1'·v1 = C' + l4·th2'·v2,  v = [-sin(th); cos(th)]
%   其中 A' = l1·alpha'·[-sin(alpha); cos(alpha)]
%         C' = l2·beta' ·[-sin(beta) ; cos(beta) ]
%   消去被动角速度 th1', th2', 即得  B' = J·[alpha'; beta']
%
% 结论 (已由 Python sympy 验证, 与 C 代码 fivebar_jacobian 一致):
%              1        [  sin(th2)·s1   -sin(th1)·s2 ]
%  J = ----------------- [                             ]
%      sin(th2 - th1)    [ -cos(th2)·s1    cos(th1)·s2 ]
%   s1 = l1·sin(th1-alpha),  s2 = l2·sin(th2-beta)

clear; clc;

syms alpha beta th1 th2 l1 l2 l3 l4 real
syms ad bd th1d th2d xbd ybd real   % 各量对时间导数

%% ---- 1. 微分后的闭环约束 ----
eq1 = xbd == -l1*sin(alpha)*ad - l3*sin(th1)*th1d;   % B' 的 x 分量, 经 A 链
eq2 = ybd ==  l1*cos(alpha)*ad + l3*cos(th1)*th1d;   % B' 的 y 分量, 经 A 链
eq3 = xbd == -l2*sin(beta)*bd  - l4*sin(th2)*th2d;   % B' 的 x 分量, 经 C 链
eq4 = ybd ==  l2*cos(beta)*bd  + l4*cos(th2)*th2d;   % B' 的 y 分量, 经 C 链

%% ---- 2. 消去 th1d, th2d, 解出 [xbd; ybd] = J·[ad; bd] ----
sol = solve([eq1 eq2 eq3 eq4], [xbd ybd th1d th2d]);
J = [diff(sol.xbd, ad), diff(sol.xbd, bd);
     diff(sol.ybd, ad), diff(sol.ybd, bd)];
J = simplify(J);

%% ---- 3. 与 C 代码闭式对比 ----
s1 = l1*sin(th1 - alpha);
s2 = l2*sin(th2 - beta);
Jc = 1/sin(th2 - th1) * [ sin(th2)*s1,  -sin(th1)*s2;
                         -cos(th2)*s1,   cos(th1)*s2 ];

fprintf('推导得到的 J 与 C 代码闭式之差 (应全为 0):\n');
disp(simplify(J - Jc))

%% ---- 4. 代入实际杆长, 显示数值形式 ----
Jnum = simplify(subs(Jc, [l1 l2 l3 l4], [210 210 250 250]));
fprintf('代入杆长 l1=l2=210, l3=l4=250 (mm) 后的 J:\n');
pretty(Jnum)

%% ---- 5. 奇异条件 ----
fprintf('奇异条件: sin(th2-th1)=0  => 两被动杆 AB,CB 共线, det(J)=0\n');
fprintf('det(J) = l1·l2·sin(th1-alpha)·sin(th2-beta)/sin(th2-th1)\n');
fprintf('另两种奇异: sin(th1-alpha)=0 (OA,AB 共线), sin(th2-beta)=0 (OC,CB 共线)\n');
