# WheelLeg — 双轮腿机器人固件

RoboMaster 轮腿机器人(双 DM8009 腿电机 ×2 + 3508/C620 轮毂 ×2),STM32H723 + Keil MDK。
设计架构：等效传动 FK/J + VMC 极坐标力 + 单侧 LQR 平衡 + 差速阻尼 + 横滚/防劈叉环。当前源码的任务创建关系仍需核对，见下文。

## 目录结构

```
WheelLeg/
├── README.md             本文件
├── WheelLeg.ioc          CubeMX 工程配置
├── MDK-ARM/              Keil 工程与编译产物
├── Core/                 CubeMX 生成代码(时钟/外设/FreeRTOS)
├── Drivers/ Middlewares/ HAL 库与中间件
├── Algorithm/            核心控制算法
│   ├── balance.c/h       平衡环:每腿角度分别计算，共用轮速与位移状态
│   ├── VMC.c/h           虚拟模型控制:正解/极坐标力/Jᵀ力映射
│   ├── controller.c/h    PID 工具
│   ├── QuaternionEKF.*   IMU 四元数 EKF 姿态估计
│   └── kalman_filter.*   卡尔曼滤波
├── Kinematics/           五连杆运动学(正逆解/雅可比/力映射,已三重验证)
├── Tasks/                FreeRTOS 任务(左右腿控制/INS 姿态)
├── Applications/         设备驱动(DM8009 MIT/DM3508+C620/遥控 DBUS)
├── Bsp/ Devices/ Timer/  板级支持
├── DynamicDesign/        动力学设计与验证(离线,见下)
└── tmp/                  临时调试产物(可随时清理)
```

## 关键约定(改代码前必读)

- **单位**:运动学层 mm / N·mm;控制层 m / N·m,下发 DM 电机时 ÷1000
- **坐标系**:从车辆左侧看，车头在画面左边。FK/VMC 的局部坐标以髋部等效基座 O 为原点，`+x` 朝车头（画面左）、`+y` 朝下，B 是轮心；腿轴沿局部 `+y` 时 `B=(0,+L_t)`，实际平衡角可以非零。动力学另用世界坐标 `+X` 朝车头、`+Z` 朝上，世界 `Z` 与 FK 的局部 `y` 不能直接混用。模型 θ 正=髋前移、φ 正=抬头、T 正=轮前滚、Tp 正=给机体抬头力矩
- **编码**:C 文件为 GBK(中文注释,Keil 环境);.md 为 UTF-8
- **符号链**曾做过实机检查；当前工作区与先前站稳时的源码版本不一致，不能直接沿用那次结论。改动任何 SIGN 宏前，应先核对当前烧录版本和反馈方向

## 设计文档(DynamicDesign/)

| 文件 | 内容 |
|---|---|
| 串联腿偏心质心动力学与LQR状态空间.md | 动力学建模:A/B 矩阵、平衡点、线性化(固件 K 的来源) |
| 串联腿动力学完整推导附录.md | 推导附录 |
| 方案.md | 控制架构、通道分工、调试里程碑 |
| continuous_lqr_cli.py | LQR 增益计算 CLI(参数更新后重跑生成新 K) |
| serial_leg_dynamics.py / _state_space.m | 模型生成、验证与状态空间导出 |
| fivebar_*.m / verify_jacobian.* | 五连杆运动学与雅可比验证套件 |
| audit_control_feedback.py | 控制反馈审计工具 |
| tests/ | 离线控制测试(力矩分配等) |

## 控制流与当前源码状态

```
编码器 → 等效五连杆正解/J → 每腿 LQR(各自 θ，共用 x/v)输出 T/Tp
       → 腿长力 + 髋力矩的极坐标力分解 → τ = JᵀF → DM8009
       → 差速阻尼(Δv)叠加两轮 → 3508/C620
       → 横滚 ±ΔF、防劈叉 ±Tp 叠加
```

当前 `Core/Src/freertos.c` 只创建了 `LLeg_task`，而 `Tasks/Leg_Tasks.c` 中的右腿控制写在独立的 `RLeg_task`。左腿任务又读取 `bal_right.T_Nm`，所以这份工作区源码不能等同于此前能站稳的烧录版本。此处仅记录核查结果，没有改动任务或电机输出代码。
