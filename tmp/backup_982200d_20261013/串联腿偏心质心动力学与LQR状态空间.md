# 串联腿偏心质心：状态方程与 LQR 矩阵

本文依次给出状态、静态平衡点、线性状态方程及其系数通式。先保留字母，不把质量、长度等数值代入矩阵。所有公式是普通文本，VS Code 不需要 LaTeX 插件。原始手写式、消元步骤、参数来源、代码位置和验证记录保存在[完整推导附录](Matlab/串联腿动力学完整推导附录.md)。

2026-10-11 实机接入与构型更正见[控制闭环排查与修正](控制闭环排查与修正.md)。仓库五连杆是串联腿的传动等效图，AB 不是实物杆，B 是轮心；不能用等效杆判断电机定子的真实姿态。此更正不改变本文固定构型的三刚体方程，`I_p` 仍为单腿绕自身质心惯量。

## 1. 构型、方向和符号

按**单侧**建模：一只轮 + 一条腿 + 半个机体。轮心为 W，合并后的基座转轴为 H，腿质心为 Gp，机体质心为 Gm。机体完整质量和惯量若分别为 `M_body`、`I_M_body`，本模型使用 `M=M_body/2`、`I_M=I_M_body/2`；`m_p` 和 `I_p` 已是一条腿的数据，不再除以二。输入 `T`、`T_p` 是该侧力矩。

世界坐标：`X` 向前、`Z` 向上。`theta` 是 W→H 相对竖直的倾角，H 前移为正；`phi` 是机体角，采用下列 Gm 位置定义其正方向。轮心高度的常数 R 已从位置表达式省略。

```text
e = (sin(theta), cos(theta))          # 沿 W→H
n = (cos(theta), -sin(theta))         # 腿的正法向

a = L*sin(theta) + y*cos(theta)      # W→Gp 的水平分量
b = L*cos(theta) - y*sin(theta)      # W→Gp 的竖直分量
h = L_t*sin(theta)                    # W→H  的水平分量
z = L_t*cos(theta)                    # W→H  的竖直分量

W  = (x, 0)
Gp = (x+a, b)
H  = (x+h, z)
Gm = (x+h-l*sin(phi), z+l*cos(phi))
```

`L` 是 W→Gp 沿腿轴的投影；`y` 是其有符号横向偏移。已确认 `y<0` 表示腿竖直时质心在轮心后方。`L_t` 为 W→H 距离，`l` 为 H→Gm 距离。θ、φ 用 rad；长度用 m；输入用 N·m。

手写笔记中的反三角函数与上面的 a、b 完全相同：

```text
rho_W  = sqrt(L^2+y^2)
beta_W = atan(y/L)

a = rho_W*sin(theta+beta_W) = L*sin(theta)+y*cos(theta)
b = rho_W*cos(theta+beta_W) = L*cos(theta)-y*sin(theta)

rho_H  = sqrt((L_t-L)^2+y^2)
beta_H = atan(y/(L_t-L))

h-a = rho_H*sin(theta-beta_H)
z-b = rho_H*cos(theta-beta_H)
```

这里展开用的是三角恒等式，没有对角度作小角度近似，偏心量 `y` 仍完整保留。四象限实现可把 `atan(y/L)` 写作 `atan2(y,L)`；本组几何中 `L>0`、`L_t-L>0`。

**输入关系：** `a`、`b` 只是由 `L`、`y` 和当前 `theta` 算出的中间量，无需另行测量或手动填入。下文 `H`、`r`、`theta_e`、`A`、`B` 都使用这些定义。把某一构型的 `L`、`y` 连同 `L_t`、质量、惯量等参数送入计算后，程序才产生该构型的数值矩阵；当前写出的公式本身没有预先代入表中的数字。不同腿长时 `L`、`y` 可能变化，不能只改 `L_t` 而沿用旧构型的另外两个值。

## 2. 先求静态平衡点

定义状态 `xi` 与输入 `u`：

```text
xi = [theta, theta_dot, x, x_dot, phi, phi_dot]^T     (6x1)
u  = [T, T_p]^T                                       (2x1)
```

先不令 θ 为零。静止在水平地面时，速度和加速度均为零，三条配平方程是：

```text
T_e = 0
T_p_e = -M*g*l*sin(phi_e)

(m_p*L+M*L_t)*sin(theta_e) + m_p*y*cos(theta_e)
    = M*l*sin(phi_e)
```

定义 `c=m_p*L+M*L_t`、`d=m_p*y`，近直立支可写为：

```text
theta_e = asin(M*l*sin(phi_e)/sqrt(c^2+d^2)) - atan2(d,c)

xi_e = [theta_e, 0, x_e, 0, phi_e, 0]^T
u_e  = [0, -M*g*l*sin(phi_e)]^T
```

若要求**机体水平**，令 `phi_e=0`，于是：

```text
theta_e = atan2(-m_p*y, m_p*L+M*L_t)
T_e = 0
T_p_e = 0
```

因为 `y<0`，一般得到 `theta_e>0`。位置 `x_e` 可自行选定，因为模型不依赖绝对水平位置。

## 3. 线性状态方程

LQR 仍需线性化；并联腿也一样。区别是本模型先由上节求出**非零的 θ 平衡角**。定义相对平衡点的偏差：

```text
delta_xi = xi - xi_e
delta_u  = u - u_e
```

保留一阶小量后，各行状态方程为：

```text
d(delta_theta)/dt = delta_theta_dot

delta_theta_ddot = A1*delta_theta + A2*delta_phi
                   + B1*delta_T + B2*delta_T_p

d(delta_x)/dt = delta_x_dot

delta_x_ddot     = A3*delta_theta + A4*delta_phi
                   + B3*delta_T + B4*delta_T_p

d(delta_phi)/dt = delta_phi_dot

delta_phi_ddot   = A5*delta_theta + A6*delta_phi
                   + B5*delta_T + B6*delta_T_p
```

其中 `d(...)/dt` 表示对时间求导，`delta_theta_dot`、`delta_x_dot` 和 `delta_phi_dot` 是状态向量中的角速度或速度偏差。上面六行合起来就是连续时间方程 `delta_xi_dot=A*delta_xi+B*delta_u`，完整矩阵形式为：

```text
delta_xi_dot = A * delta_xi + B * delta_u

    [0   1  0  0  0   0 ]          [0   0 ]
    [A1  0  0  0  A2  0 ]          [B1  B2]
A = [0   0  0  1  0   0 ]      B = [0   0 ]
    [A3  0  0  0  A4  0 ]          [B3  B4]
    [0   0  0  0  0   1 ]          [0   0 ]
    [A5  0  0  0  A6  0 ]          [B5  B6]
```

这里包含机体角 `phi` 和髋力矩 `T_p`，故为 **6×6 的 A、6×2 的 B**。下节逐项给出 A1～A6、B1～B6 的全符号式。

## 4. 各个系数由哪些物理量决定

先定义等效轮平动质量 `m_0=m_w+I_w/R^2`。把第 1 节的位置代入牛顿—欧拉方程并消去内力后，非线性方程是：

```text
q = [theta, x, phi]^T
H(q)*q_ddot = r(q,q_dot) + D*u
```

质量矩阵 `H` 的元素为：

```text
H = [h11  h12  h13
     h12  h22  h23
     h13  h23  h33]

h11 = I_p + m_p*(L^2+y^2) + M*L_t^2
h12 = m_p*(L*cos(theta)-y*sin(theta)) + M*L_t*cos(theta)
h13 = -M*L_t*l*cos(theta+phi)
h22 = m_0+m_p+M
h23 = -M*l*cos(phi)
h33 = I_M+M*l^2

D = [-1    1
     1/R   0
      0    1]
```

非线性项 `r=[r1,r2,r3]^T` 完整保留重力和速度平方项：

```text
r1 = g*(m_p*a+M*h) - M*L_t*l*sin(theta+phi)*phi_dot^2
r2 = (m_p*a+M*h)*theta_dot^2 - M*l*sin(phi)*phi_dot^2
r3 = M*g*l*sin(phi) - M*L_t*l*sin(theta+phi)*theta_dot^2
```

在 `(theta_e, phi_e)` 处计算 `H_e=H(theta_e,phi_e)`。定义两项重力导数：

```text
b_e     = L*cos(theta_e)-y*sin(theta_e)
k_theta = g*(m_p*b_e+M*L_t*cos(theta_e))
k_phi   = M*g*l*cos(phi_e)
```

矩阵与系数的最短全符号写法为：

```text
F = inverse(H_e)*diag(k_theta, 0, k_phi)
G = inverse(H_e)*D

A1=F11   A2=F13   A3=F21   A4=F23   A5=F31   A6=F33
B1=G11   B2=G12   B3=G21   B4=G22   B5=G31   B6=G32
```

若需要把每个系数写成单独的分式，定义 `H_e` 的六个不同元素 `h11...h33`，并令：

```text
C11 = h22*h33-h23^2          C12 = h13*h23-h12*h33
C22 = h11*h33-h13^2          C13 = h12*h23-h13*h22
C33 = h11*h22-h12^2          C23 = h12*h13-h11*h23

Delta = h11*C11+h12*C12+h13*C13
```

于是**矩阵中的 12 个系数**是：

| A 中的系数 | 字母表达式 | B 中的系数 | 字母表达式 |
|---|---|---|---|
| A1 | `C11*k_theta/Delta` | B1 | `(-C11+C12/R)/Delta` |
| A2 | `C13*k_phi/Delta` | B2 | `(C11+C13)/Delta` |
| A3 | `C12*k_theta/Delta` | B3 | `(-C12+C22/R)/Delta` |
| A4 | `C23*k_phi/Delta` | B4 | `(C12+C23)/Delta` |
| A5 | `C13*k_theta/Delta` | B5 | `(-C13+C23/R)/Delta` |
| A6 | `C33*k_phi/Delta` | B6 | `(C13+C33)/Delta` |

`Delta` 是 `H_e` 的行列式。实际求解使用 `H_e\...` 或线性方程求解器，不必显式求逆。各系数中出现的 `theta_e` 已由第 2 节的平衡条件确定；此处仍未代入数值。

## 5. 非零平衡角具体改变了什么

1. **增加配平步骤。** 先解 `theta_e`，若 `phi_e` 不为零，还需加入 `T_p_e` 偏置。
2. **反馈使用偏差。** 控制律写作 `u=u_e-K*(xi-xi_e)`。以机体水平为目标时，θ 的目标是 `theta_e`。
3. **系数在正确点取值。** `h12、h13、k_theta` 等包含 `sin(theta_e)`、`cos(theta_e)`。`y` 也进入质量矩阵和重力导数。

线性化本身没有增加状态；仍是 6 个状态、2 个输入。若直接在 `theta=0, phi=0, u=0` 展开，由于它不是偏心腿的静态平衡点，线性模型会多出**非零常数项**；把那个矩阵直接用于零目标 LQR 会遗漏静态偏差。

## 6. 状态转移矩阵与实现边界

用户已确定采样周期 `T_s=1 ms=0.001 s`。第 3 节的 `A` 是连续时间系统矩阵；**一步状态转移矩阵**是 `A_d=Phi(0.001)`。假设每个周期内输入保持不变（零阶保持），精确离散式为：

```text
A_d = expm(0.001*A)
B_d = integral(tau=0..0.001, expm(A*tau)*B d_tau)

delta_xi[k+1] = A_d*delta_xi[k] + B_d*delta_u[k]
```

对含位置积分状态的 A，使用分块矩阵指数同时求 A_d、B_d，避免对可能不可逆的 A 求逆：

```text
E = expm(0.001 * [ A(6x6)   B(6x2)
                   0(2x6)   0(2x2) ])

A_d = E 的第 1～6 行、第 1～6 列
B_d = E 的第 1～6 行、第 7～8 列
```

[serial_leg_state_space.m](Matlab/serial_leg_state_space.m) 已实现这一计算。传入本构型的参数结构 `p` 后，可调用 `serial_leg_state_space(p,phi_e,0.001)`。此处仍保留 A/B 的全符号形式，所以 A_d/B_d 也是确定计算式，而非已经代入物理参数的数值矩阵。当前决定使用**连续 A/B 设计 K**；1 ms 是控制器执行周期。仓库当前控制代码仍使用五连杆等效传动；LQR 输出的 N·m 与 VMC 的 N·mm、电机协议整数之间需要另做单位和方向转换。

### 用哪一组矩阵求 LQR 增益

本次采用 `K=lqr(A,B,Q,R_u)`，在 MCU 上每 1 ms 按 `u=u_e-K*(xi-xi_e)` 更新一次输出。这个选择使用的是连续 A/B；上面的 A_d/B_d 保留为离散仿真或以后改用 `dlqr` 的接口。`lqr` 与 `dlqr` 对应各自的时间模型，不混用矩阵。

### 输入参数并计算 K

[continuous_lqr_cli.py](Matlab/continuous_lqr_cli.py) 只依赖 Python 和 NumPy。打开仓库根目录的终端运行：

```powershell
python Matlab/continuous_lqr_cli.py
```

脚本会逐项询问轮半径、腿长、`L`、`y`、`l`、单轮/单腿质量与惯量、**未除以二的整机体质量和惯量**、`g`、目标机体平衡角 `phi_e`（rad），以及 `Q` 的 6 个对角线权重和 `R_u` 的 2 个对角线权重。脚本内部将整机体质量、惯量各除以二；单轮和单腿数据保持原值。所有长度用 m、质量用 kg、惯量用 kg·m²；`y` 向后取负。Q 的状态顺序是 `[theta,theta_dot,x,x_dot,phi,phi_dot]`，R_u 的输入顺序是 `[T,T_p]`。`R_u` 的权重必须为正。

脚本计算平衡点、连续 A/B、2×6 的 K，并显示闭环极点及 Riccati 残差。可加 `--save Matlab/my_lqr_result.json` 保存本次输入和结果。另有 `--example`，只用原仓库参数及《方案.md》给出的起始权重演示计算，**输出不是最终实机增益**。

计算接口的输入也遵循上述关系：[serial_leg_state_space.m](Matlab/serial_leg_state_space.m) 从传入的 `p.L`、`p.y` 等参数计算平衡点和 A/B；[serial_leg_dynamics.py](Matlab/serial_leg_dynamics.py) 带有一组当前构型的默认数值，运行时会代入它们。修改构型应提供新测得或拟合的 `L`、`y`、`I_p`，再重新计算。

参数采集记录和已做的消元核对，见[完整推导附录](Matlab/串联腿动力学完整推导附录.md)。

## 7. 从车辆左侧核对反馈和力矩方向

观察约定：站在车辆左侧看，**车头在画面左边**，因此画面左是车辆前方、画面右是车辆后方。右手定则需要先指定正轴：本节补全世界右手系为 `X` 向车前、`Y` 向车左、`Z` 向上；`+Y` 朝向观察者，绕 `+Y` 的正转动在画面上是逆时针。这里的大写 `Y` 是车体横向轴，不能与腿质心偏移量小写 `y` 混用。

当前模型保留原笔记的广义坐标定义：`theta` 增大时髋轴向前，实际腿角速度沿 `+Y`；`phi` 增大时机体质心向后，实际机体角速度沿 `-Y`。因此当前模型的两个角并非都以同一根 `+Y` 轴为正。这不违反右手定则，但接入 IMU 时必须换算，不能直接把“逆时针为正”的机体角当成 `phi`。

### 7.1 先分清力矩作用在哪个物体上

| 模型量 | 按统一 `+Y` 轴表示 | 从车辆左侧看到的正方向 |
|---|---|---|
| `theta_dot` | 腿的绝对角速度 `omega_leg_Y=theta_dot` | 逆时针；W→H 向车前倾 |
| `phi_dot` | 机体绝对角速度 `omega_body_Y=-phi_dot` | 顺时针；机体顶部向后倾 |
| 正轮力矩 `T` | 作用在轮上为 `+T`，反作用在腿上为 `-T` | 轮受逆时针力矩，腿受顺时针反力矩 |
| 正模型髋力矩 `T_p` | 作用在腿上为 `+T_p`，反作用在机体上为 `-T_p` | 腿受逆时针力矩，机体受顺时针反力矩 |

力矩方向不等于自由整车上某个物体的净角加速度方向，因为还存在重力、接触力和耦合惯性。上表描述执行器施加的力矩。

将力矩与角速度相乘，还能核对整套符号。纯滚动时 `omega_wheel_Y=x_dot/R`，执行器功率为：

```text
P_wheel_motor = T * (x_dot/R - theta_dot)
P_hip_motor   = T_p * (theta_dot + phi_dot)

P_total = (-T+T_p)*theta_dot + (T/R)*x_dot + T_p*phi_dot
```

所以按 `[theta, x, phi]` 顺序写广义驱动力，就是 `[-T+T_p, T/R, T_p]`。腿方程和机体方程都出现 `+T_p`，是因为 `theta` 与 `phi` 的实际转动正向相反；作用力矩和反作用力矩仍然相反。

### 7.2 逐层核对实物反馈与输出

先断电手动核对反馈，再支撑车身、轮离地、限制力矩逐通道点动。测试时暂时去掉 LQR、滚转、防劈叉和腿长环等叠加指令，使用明确的测试输入；`VMC_ForceCalc` 内部会重新计算 `F_axis_N`，不能只在调用前给该字段赋值就认为腿长环已关闭。若重力使运动趋势不清楚，应在相同起始姿态比较正、负小指令造成的初始速度变化，不能以最终姿态判定力矩方向。

| 观察动作或给定指令 | 正确的状态/初始运动方向 |
|---|---|
| 断电手动让轮心相对髋轴向后（画面右） | 两腿换算后的 `theta_rel_body_rad=leg_axis_angle_rad-π/2` 应增大；机体固定时 `theta` 应增大 |
| 断电手动让机体上方向后倾（画面右） | `phi` 应增大；同时按 `theta=theta_rel_body_rad-phi` 检查组合角是否符合实物 |
| 悬空给正 `F_axis_N`，机体用支架固定 | O→B 的腿长 `L_t_mm` 应增加 |
| 悬空给正 VMC `T_p_Nmm`，机体用支架固定、只观察初始趋势 | 轮心应相对髋轴向后（画面右），`leg_axis_angle_rad` 和 `theta_rel_body_rad` 应增大；在腿接近竖直时 `n0=(-e0_y,e0_x)` 近似指向后方 |
| 两轮悬空，给正模型轮力矩 `T` | 两只轮子都应按车辆前进的方向旋转；从车辆左侧看是逆时针 |

其中机体角的检查还应覆盖角速度：手动让机体逆时针转动，统一右手系的 `omega_body_Y` 应为正，而送入现有 LQR 的 `phi_dot` 应为负。实际选用哪个 IMU 轴、是否取反，须由安装和该动作的读数确认。轮子向前滚动时，左右换算后的 `x_dot` 均应为正。

必须把**测量方向**与**驱动方向**分开验。先确认单个驱动器的正原始力矩，会使它自己的原始编码器角速度向正方向变化。右腿当前代码读入 `q_shared=-q_raw`，再把雅可比输出变为 `tau_raw=-tau_shared`。下式先把力矩统一为 N·m、位置统一为 m；代码中的 N·mm 另需除以 1000：

```text
q_shared_dot = -q_raw_dot
sum(tau_raw_i * q_raw_dot_i)
    = sum(tau_shared_i * q_shared_dot_i)
    = F dot B_dot
```

对两个关节分别求和后，这个关系说明“双向取反”在代数上是匹配的。它不能证明实际驱动器的正力矩与编码器正角速度同向，也不能证明 IMU 俯仰正方向正确；需要上表的断电手动运动和低力矩点动。左腿无此镜像变换，两腿**换算后的** `theta_rel_body_rad`、`L_t_mm`、`theta` 必须使用相同的世界前向定义。

VMC 共用局部坐标为“前、下”，`leg_axis_angle_rad=atan2(B.y,B.x)` 的正向也是绕车左轴，故 `theta_rel_body_rad=leg_axis_angle_rad-pi/2=theta+phi`，得到 `theta=theta_rel_body_rad-phi`。`VMC_force_polar_to_cart` 中正 `T_p_Nmm` 沿增大 `leg_axis_angle_rad` 的切向作用，与模型对腿施加的正 `T_p` 对应；模型 N·m 输入到 VMC 的 N·mm 量应乘以 1000。右腿的镜像符号已在关节输入和输出边界处理，不应仅因是右腿又额外把模型 `T_p` 取反。

轮命令另有一次镜像：`WheelTorque_Send` 向左轮发送输入乘以 3000 的电流整数，向右轮发送相反数，0x200 数据按高字节在前打包。2026-10-11 当前 `LLeg_task` 已统一计算两腿并每帧调用该函数一次。`balance.c` 先以 `LQR_WHEEL_SIGN=-1` 把模型轮力矩转为驱动接口值；因此 Watch 中 `bal.T_Nm` 的正号表示模型负力矩，不能混读。用户已确认手推车向后时左轮 RPM 为正、右轮为负，当前观测符号为左负、右正。3000 是沿用的力矩到协议值系数，绝对力矩幅值仍有标定误差。

用这次 `Q=diag(1,1,1,1,1,1)`、`R_u=diag(5,5)` 算出的 K，控制律为 `u=u_e-K*(xi-xi_e)`。只看单一正误差时，它应给出以下**计算指令趋势**，供台架记录核对：

| 单一误差为正 | 计算出的轮力矩 `T` | 计算出的髋力矩 `T_p` | 直观目的 |
|---|---:|---:|---|
| `theta-theta_e > 0`，髋轴在轮心前方 | 正 | 负 | 轮追向前方，同时腿回摆 |
| `phi-phi_e > 0`，机体质心向后 | 负 | 负 | 轮向后追机体质心，同时机体回正 |

此表检验的是**指令计算符号**，实际自由运动还受轮、腿、机体耦合和支撑约束影响，不能只看整车第一瞬间的动作就判错。上电检查应由支架承重、轮离地、限制电流或力矩、逐只点动开始；先关闭 LQR、滚转和防劈叉等叠加输出，最后再在受控条件下闭合反馈。符号和单位确认前不要直接使用完整 K 站立。

### 7.3 若接口统一使用绕车左轴的正角度与机体力矩

可以定义新的机体角 `psi=-phi`，并把作用在机体上的正轴力矩定义为 `tau_body=-T_p`。于是 `psi`、`theta` 都以画面逆时针为正，`tau_body>0` 表示给机体逆时针力矩。此时需要一起变换状态、平衡点、输入和矩阵，不能只改变量名或单独翻转一个输出：

```text
xi_new = [theta, theta_dot, x, x_dot, psi, psi_dot]^T = S*xi
u_new  = [T, tau_body]^T = U*u
S = diag(1, 1, 1, 1, -1, -1)
U = diag(1, -1)

A_new = S*A*S
B_new = S*B*U
K_new = U*K*S
xi_e_new = S*xi_e
u_e_new = U*u_e

u_new = u_e_new - K_new*(xi_new-xi_e_new)
theta = alpha + psi
T_p_Nmm = -1000*tau_body
```

这里只记录等价换算；现有 Python/MATLAB 脚本、数值 K 和前面各节仍使用原来的 `phi`、`T_p` 定义。保留原模型时，在接口处按 `phi=-psi`、`phi_dot=-omega_body_Y` 换算即可。

### 7.4 建模量与当前 VMC 字段

| 建模量或含义 | VMC 字段 | 关系与单位 |
|---|---|---|
| 单侧轮心到髋轴长度 `L_t` | `L_t_mm` | `L_t_mm=1000*L_t`；VMC 实时测量，当前 A/B 在某个固定构型取值；`L_t_set_mm` 是腿长目标 |
| 腿的世界角 `theta` | `theta_rel_body_rad` | 该字段是相对机体角，`theta=theta_rel_body_rad-phi`，不可直接作为 LQR 的 `theta` |
| 机体世界角 `phi` | 无 | 由 IMU 按本文正向换算后送入状态估计 |
| 模型髋力矩 `T_p` | `T_p_Nmm` | `T_p_Nmm=1000*T_p`；正方向与模型对腿的正力矩相同 |
| 支撑腿的虚拟轴向力 | `F_axis_N` | 沿髋轴指向轮心，不是模型中任何一个惯性系竖直力 |
| FK 局部几何角 | `leg_axis_angle_rad` | 从 FK 的车前轴量到 H→W，`leg_axis_angle_rad=pi/2+theta_rel_body_rad` |

`F_fk_x_N`、`F_fk_y_N` 是 FK 局部“前、下”坐标的力；不可把后者当成世界竖直向上的力。`joint_a_angle_rad`、`joint_b_angle_rad` 是两个电机输入角，不是模型的 `theta`、`phi`。模型轮力矩 `T` 与轮位移 `x` 仍不在 VMC 内计算。

2026-10-11 当前 `VMC_init` 把 `L_t_set_mm` 初始化为 212.48 mm，与上面示例 A/B 的固定构型长度相同。`balance.c` 使用两腿平均世界角、共享轮速和一个位置积分；此模型只描述同步纵向运动，两轮速度差单独用负反馈阻尼处理。`THETA_E_RAD` 保留本次读到的传感器目标值 −0.065042 rad，不因它与 `y` 的数值相同就认为两者等价。固件坐标平衡角与理论平衡角之间的零点偏置须单独核实。
