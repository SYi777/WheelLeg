# -*- coding: utf-8 -*-
import io, re, sys

def strip(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    return t

def load_raw(path):
    return io.open(path, encoding='gbk', newline='').read()

def norm(t):
    return t.replace('\r\n', '\n')

def apply(path, repls):
    raw = load_raw(path)
    text = norm(raw)
    for old, new in repls:
        n = text.count(old)
        if n != 1:
            raise SystemExit('ANCHOR COUNT %d for %r in %s' % (n, old[:60], path))
        text = text.replace(old, new, 1)
    # 保持文件原有 CRLF
    new_text = text.replace('\r\n', '\n').replace('\n', '\r\n')
    old_n = norm(io.open('tmp/comment_audit_originals/' + path, encoding='gbk').read())
    new_n = norm(new_text)
    assert strip(old_n) == strip(new_n), '代码被改动, 禁止写盘: ' + path
    new_text.encode('gbk')  # 确保 GBK 可编码
    io.open(path, 'w', encoding='gbk', newline='').write(new_text)
    print('OK', path)

# ===================== VMC.h =====================
h_repls = [
# 大方块说明: 补输出层约定
(""" *    (F_fk_x_N/F_fk_y_N/F_tangent_N 存于 vmc_t 仅为 Keil watch 调试观测, 语义上仍是内部量)
 * ==================================================================== */""",
 """ *    (F_fk_x_N/F_fk_y_N/F_tangent_N 存于 vmc_t 仅为 Keil watch 调试观测, 语义上仍是内部量)
 *
 *  输出 (关节层): tau_a/tau_b [N*mm] 经 VMC_TorqueAllocate() 限幅分配后下发
 * ==================================================================== */"""),

# 分节标题: 补充限幅标志说明
("// 站立力矩截断变量",
 "// 分配与限幅 (站立力矩截断变量)"),

# 三个新字段注释: 写清变量意义/单位/调试用途
("/* 截断前的轴向力分量 */",
 "/* 截断前轴向力分量: 弹簧阻尼力/重力补偿 [N], 调试观测 */"),
("/* 分配后的可行极坐标指令 */",
 "/* 分配后可行极坐标指令: 腿轴力/髋力矩 [N]/[N*mm], 实际下发 */"),
("/* 分配前力矩 */",
 "/* 分配前关节请求力矩 [N*m], 由 tau_a/tau_b 换算 */"),

# 限幅标志: 行内补注释(紧跟分号, 不引入额外空白, 保证代码逐字不变)
("uint8_t axis_force_limited, hip_moment_limited;",
 "uint8_t axis_force_limited, hip_moment_limited;// 限幅标志: 1=F_axis 被夹 / 髋力矩被等比缩小"),

# 上一周期缓存: 补注释
("float last_L_t_mm;",
 "float last_L_t_mm;// 上一周期腿长 [mm]"),
("float last_leg_axis_angle_rad;",
 "float last_leg_axis_angle_rad;// 上一周期腿轴极角 [rad]"),

# F_axis_N 为分配前期望值; F_tangent 分配后被覆盖
("// 期望沿腿轴力 [N] = ",
 "// 期望沿腿轴力 [N] (分配前) = "),
("// 切向力 [N] = T_p_Nmm/L_t_mm (沿 n0)",
 "// 切向力 [N] = T_p_Nmm/L_t_mm (沿 n0, 分配后为实际值)"),

# 输出段
("// 输出[N·mm]", "// 输出"),

# 力坐标变换声明
("// 力坐标变换: 极坐标 (F_axis_N,T_p_Nmm) <-> 直角坐标 (F_fk_x_N,F_fk_y_N), 与 B 同坐标系",
 "// 力坐标变换: 极坐标 (F_axis_N,T_p_Nmm) <-> 直角坐标 (F_fk_x_N,F_fk_y_N), 均以 B 点为原点"),
]
apply('Algorithm/VMC.h', h_repls)

# ===================== VMC.c =====================
c_repls = [
# 文件头大方块: 删除纯英文重复句, 统一中文, 补分配说明
(""" *  VMC - Virtual Model Control (虚拟模型控制)
 *  力坐标约定见 VMC.h 头部: 对外极坐标 (F_axis_N, T_p_Nmm), 直角坐标 (F_fk_x_N, F_fk_y_N)
 *  只在本模块内部使用 (喂给直角坐标雅可比 fivebar_force), 不出边界。
 *  Force convention: see VMC.h - polar (F_axis_N, T_p_Nmm) at the boundary;
 *  Cartesian (F_fk_x_N, F_fk_y_N) is internal only (feeds fivebar_force).""",
 """ *  VMC - 虚拟模型控制
 *  力坐标约定见 VMC.h 头部: 对外极坐标 (F_axis_N, T_p_Nmm),
 *  直角坐标 (F_fk_x_N, F_fk_y_N) 只在本模块内部使用 (喂给 fivebar_force)。
 *  关节力矩经 VMC_TorqueAllocate() 按 TORQUE_LIMIT 限幅分配后下发。"""),

# VMC_init 单行注释 -> 标准函数注释
("// 初始化本腿 VMC 状态和站立腿长目标",
 """/**
 * @brief  初始化本腿 VMC 状态与站立腿长目标
 * @param  vmc  本腿 VMC 状态
 * @return 无
 *
 * 清零力与力矩指令, 正解状态置为 FK_NO_SOLUTION。
 */"""),

# 新增字段初始化注释
("0.0f;   // ← 新增", "0.0f;   // 分配后指令清零"),
("0U; // ← 新增", "0U; // 限幅标志清零"),

# StateUpdate 补充 first_flag 用途
(""" * 局部坐标 +x 朝车头、+y 朝下；角速度由相邻周期位置差计算。
 */""",
 """ * 局部坐标 +x 朝车头、+y 朝下；角速度由相邻周期位置差计算。
 * FK 失败时置 first_flag, 恢复后用当前值重置差分基准, 避免 dt 尖峰。
 */"""),

# StateUpdateWithVelocity 补充说明
(""" * @param  dqb  第 2 个驱动角速度 [rad/s]
 * @param  dt  本次控制周期 [s]
 * @return 无
 */""",
 """ * @param  dqb  第 2 个驱动角速度 [rad/s]
 * @param  dt  本次控制周期 [s]
 * @return 无
 *
 * 用雅可比 J 把关节速度映射为轮心速度 (vx,vy), 再投影到腿轴方向与切向,
 * 得到 L_t_dot 与极角速度, 精度优于位置差分。
 */"""),

# ForceCalc 补充力链说明
(" * 腿轴力包含长度弹簧阻尼、重力补偿和横滚修正。",
 """ * 腿轴力 = 长度弹簧阻尼 + 重力补偿 + 横滚修正:
 *   F_spring  = VMC_KP*(L_t_set_mm-L_t_mm) + VMC_KD*(0-L_t_dot_mm_s)  [N]
 *   F_gravity = MASS_HALF_G/cos(theta_rel_body_rad)                    [N]
 * F_spring_N/F_gravity_N 保存截断前分量 (调试观测), F_axis_N 夹到
 * ±VMC_F_AXIS_MAX_N 后转直角坐标求 tau, 再交 VMC_TorqueAllocate() 分配。"""),

# 极坐标->直角坐标 注释
("// 将目标极坐标力转换为局部直角坐标力",
 "// 目标极坐标力 -> 局部直角坐标力"),

# 分配调用行内注释
("\t\tVMC_TorqueAllocate(vmc, TORQUE_LIMIT);",
 "\t\tVMC_TorqueAllocate(vmc, TORQUE_LIMIT);// 按 TORQUE_LIMIT 限幅分配"),

# polar_to_cart 补充公式
(" * 切向单位向量为 (-B.y, B.x)/|B|。",
 """ * 切向单位向量 n0 = (-B.y, B.x)/|B|,
 * F_cart = F_axis*e0 + F_tangent*n0, 其中 e0 = B/|B|, F_tangent = T_p/|B|。"""),

# cart_to_polar 补充公式
(""" * @param  T_p_Nmm  输出广义髋力矩 [N*mm]
 * @return 无
 */""",
 """ * @param  T_p_Nmm  输出广义髋力矩 [N*mm]
 * @return 无
 *
 * 投影: F_axis = F·e0, F_tangent = F·n0, T_p = F_tangent*|B|,
 * 其中 e0 = B/|B|, n0 = (-B.y, B.x)/|B|。
 */"""),

# 切向分量注释
("  // F.n0", "  // 切向分量 F·n0 [N]"),

# VMC_TorqueAllocate 函数文档: 四步 + 变量意义
("""/* 髋力矩优先的力/力矩可行性分配:
 * 先把 T_p 的髋力矩分量压进 TORQUE_LIMIT (必要时等比缩小),
 * 再用剩余扭矩空间求 F_axis 的可行区间并夹入, 最后重算 J^T F。
 * 输出 F_axis_applied_N / T_p_applied_Nmm 与限幅标志, 供调试观测。 */""",
 """/**
 * @brief  按关节力矩上限分配腿轴力与髋力矩, 并重算关节力矩
 * @param  vmc  本腿 VMC 状态 (输入 F_axis_N/T_p_Nmm, 输出分配结果)
 * @param  torque_limit_Nm  单关节力矩上限 [N*m]
 * @return 无
 *
 * 四步:
 * 1) 记录截断前力矩: tau_request_a_Nm/tau_request_b_Nm = 当前 tau/1000;
 * 2) 髋力矩优先: 把 T_p 产生的两关节力矩等比压进 ±torque_limit_Nm,
 *    超限时整体缩小 moment_Nm, 并置 hip_moment_limited = 1;
 * 3) 用剩余扭矩空间夹 F_axis: 由 tau_i = force_gain[i]*F + hip_torque[i]
 *    反解 |tau_i| <= torque_limit_Nm 的可行区间并夹入, 被夹时置
 *    axis_force_limited = 1, 区间为空则取 F = 0;
 * 4) 用可行力重算 J^T F: 由 F_axis_applied_N/T_p_applied_Nmm 转直角坐标,
 *    调用 fivebar_force() 写回 tau_a/tau_b。
 *
 * 记 e0 = B/|B|, n0 = (-e0.y, e0.x), 则 tau_i = force_gain[i]*F_axis + hip_torque[i]:
 *   force_gain[i]  = (J^T e0)_i / 1000      轴向力到关节力矩的增益 [N*m/N]
 *   moment_gain[i] = (J^T n0)_i / |B|       髋力矩到关节力矩的增益 (无量纲)
 *   hip_torque[i]  = moment_gain[i]*T_p     髋力矩分量产生的关节力矩 [N*m]
 * 分配后指令存 F_axis_applied_N/T_p_applied_Nmm, 供 Keil watch 调试观测。
 */"""),

# 分配器内部行内注释
("vmc->tau_request_a_Nm = vmc->fk_s.tau_a / 1000.0f;",
 "vmc->tau_request_a_Nm = vmc->fk_s.tau_a / 1000.0f;// 分配前关节力矩 [N*m]"),
("vmc->tau_request_b_Nm = vmc->fk_s.tau_b / 1000.0f;",
 "vmc->tau_request_b_Nm = vmc->fk_s.tau_b / 1000.0f;// 分配前关节力矩 [N*m]"),
("0U;\n    if (vmc->fk_s.fk.status != FK_OK || !vmc->fk_s.jac_ok",
 "0U;// 清限幅标志\n    if (vmc->fk_s.fk.status != FK_OK || !vmc->fk_s.jac_ok"),
("        vmc->F_axis_applied_N = vmc->T_p_applied_Nmm = 0.0f;",
 "        vmc->F_axis_applied_N = vmc->T_p_applied_Nmm = 0.0f;// 无效状态清零"),
("    float ex = vmc->fk_s.fk.B.x / vmc->L_t_mm;",
 "    float ex = vmc->fk_s.fk.B.x / vmc->L_t_mm;// 腿轴单位向量 e0"),
("    float ey = vmc->fk_s.fk.B.y / vmc->L_t_mm;",
 "    float ey = vmc->fk_s.fk.B.y / vmc->L_t_mm;// 腿轴单位向量 e0"),
("    float moment_Nm = vmc->T_p_Nmm / 1000.0f;",
 "    float moment_Nm = vmc->T_p_Nmm / 1000.0f;// 期望髋力矩 [N*m]"),
("    float peak = fmaxf(fabsf(hip_torque[0]), fabsf(hip_torque[1]));",
 "    float peak = fmaxf(fabsf(hip_torque[0]), fabsf(hip_torque[1]));// 两关节峰值 [N*m]"),
("        float scale = torque_limit_Nm / peak;",
 "        float scale = torque_limit_Nm / peak;// 等比缩小系数"),
("    float force_low = -FLT_MAX, force_high = FLT_MAX;",
 "    float force_low = -FLT_MAX, force_high = FLT_MAX;// 可行轴向力区间 [N]"),
("        float a = (-torque_limit_Nm - hip_torque[i]) / force_gain[i];",
 "        float a = (-torque_limit_Nm - hip_torque[i]) / force_gain[i];// tau=-limit 对应力 [N]"),
("        float b = ( torque_limit_Nm - hip_torque[i]) / force_gain[i];",
 "        float b = ( torque_limit_Nm - hip_torque[i]) / force_gain[i];// tau=+limit 对应力 [N]"),
("    float force_N = vmc->F_axis_N;",
 "    float force_N = vmc->F_axis_N;// 夹入可行区间"),
("    /* 髋力矩等比缩放后, 零轴向力总是可行 */",
 "    // 髋力矩等比缩放后, 零轴向力总是可行"),
("    vmc->axis_force_limited = fabsf(force_N - vmc->F_axis_N) > 1e-3f;",
 "    vmc->axis_force_limited = fabsf(force_N - vmc->F_axis_N) > 1e-3f;// 记录是否被夹"),
("    vmc->F_axis_applied_N = force_N;",
 "    vmc->F_axis_applied_N = force_N;// 分配后可行指令"),
("    vmc->T_p_applied_Nmm = moment_Nm * 1000.0f;",
 "    vmc->T_p_applied_Nmm = moment_Nm * 1000.0f;// 分配后髋力矩 [N*mm]"),
("    vmc->F_tangent_N = vmc->T_p_applied_Nmm / vmc->L_t_mm;",
 "    vmc->F_tangent_N = vmc->T_p_applied_Nmm / vmc->L_t_mm;// 实际切向力 [N]"),
("                   &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);",
 "                   &vmc->fk_s.tau_a, &vmc->fk_s.tau_b);// 用可行力重算关节力矩 [N*mm]"),
]
apply('Algorithm/VMC.c', c_repls)
print('ALL DONE')
