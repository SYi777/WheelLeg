"""Interactive continuous-time LQR for the fixed-shape eccentric wheel-leg model.

Usage:
    python Matlab/continuous_lqr_cli.py
    python Matlab/continuous_lqr_cli.py --example

No SciPy dependency: the continuous Riccati equation is solved through the
stable invariant subspace of its Hamiltonian matrix using NumPy.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from serial_leg_dynamics import Parameters, linearize


def ask_number(label, *, positive=False, default=None):
    while True:
        suffix = f" [{default}]" if default is not None else ""
        try:
            raw = input(f"{label}{suffix}: ").strip()
            if raw == "" and default is not None:
                value = float(default)
            else:
                value = float(raw)
            if not np.isfinite(value) or (positive and value <= 0):
                raise ValueError("value must be finite" + (" and greater than zero" if positive else ""))
            return value
        except ValueError as exc:
            print(f"输入无效：{exc}。请按提示重新输入。")


def ask_diagonal(label, count, *, positive=False):
    while True:
        try:
            raw = input(f"{label}（{count} 个数，空格或逗号分隔）: ")
            values = np.array([float(x) for x in raw.replace(",", " ").split()], dtype=float)
            if len(values) != count or not np.all(np.isfinite(values)):
                raise ValueError(f"需要恰好 {count} 个有限数")
            if np.any(values <= 0) if positive else np.any(values < 0):
                raise ValueError("R 的权重必须大于零" if positive else "Q 的权重不能为负")
            return values
        except ValueError as exc:
            print(f"输入无效：{exc}。请按提示重新输入。")


def ask_inputs():
    print("\n连续时间 LQR；一只轮 + 一条腿 + 半个机体；输入全用 SI 单位。")
    print("整机体 M_body、I_M_body 由脚本除以 2；单轮/单腿量保持原值。")
    print("Q 顺序：theta, theta_dot, x, x_dot, phi, phi_dot；R 顺序：T, T_p。\n")
    R = ask_number("轮半径 R (m)", positive=True)
    Lt = ask_number("轮心到髋轴 L_t (m)", positive=True)
    L = ask_number("轮心到腿质心的轴向投影 L (m)")
    y = ask_number("腿质心横向偏移 y (m，向后为负)")
    l = ask_number("髋轴到机体质心 l (m)")
    mw = ask_number("单轮质量 m_w (kg)", positive=True)
    mp = ask_number("单腿质量 m_p (kg)", positive=True)
    M_body = ask_number("整机体质量 M_body (kg，未除以2)", positive=True)
    Iw = ask_number("单轮绕轴惯量 I_w (kg*m^2)", positive=True)
    Ip = ask_number("单腿绕质心惯量 I_p (kg*m^2)", positive=True)
    IM_body = ask_number("整机体绕质心惯量 I_M_body (kg*m^2，未除以2)", positive=True)
    g = ask_number("重力加速度 g (m/s^2)", positive=True, default=9.81)
    phi_e = ask_number("期望机体平衡角 phi_e (rad)", default=0.0)
    q = ask_diagonal("Q 对角线", 6)
    r = ask_diagonal("R_u 对角线", 2, positive=True)
    params = Parameters(R=R, Lt=Lt, L=L, y=y, l=l, mw=mw, mp=mp,
                        M=M_body/2, Iw=Iw, Ip=Ip, IM=IM_body/2, g=g)
    return params, phi_e, q, r


def continuous_lqr(A, B, q_diag, r_diag):
    """Solve the stabilizing continuous algebraic Riccati equation."""
    Q = np.diag(q_diag)
    R_u = np.diag(r_diag)
    BRB = B @ np.linalg.solve(R_u, B.T)
    hamiltonian = np.block([[A, -BRB], [-Q, -A.T]])
    eigenvalues, eigenvectors = np.linalg.eig(hamiltonian)
    stable = np.flatnonzero(eigenvalues.real < -1e-9)
    n = A.shape[0]
    if len(stable) != n:
        raise ValueError("Hamiltonian 稳定子空间维数不等于 6；检查参数和 Q/R。")
    X = eigenvectors[:n, stable]
    Y = eigenvectors[n:, stable]
    if np.linalg.cond(X) > 1e12:
        raise ValueError("Riccati 求解病态；检查参数量纲及权重。")
    P_complex = np.linalg.solve(X.T, Y.T).T
    if np.max(np.abs(P_complex.imag)) > 1e-7 * max(1., np.max(np.abs(P_complex.real))):
        raise ValueError("Riccati 解存在明显虚部；检查参数和权重。")
    P = P_complex.real
    P = (P + P.T) / 2
    K = np.linalg.solve(R_u, B.T @ P)
    residual = A.T @ P + P @ A - P @ BRB @ P + Q
    residual_rel = np.linalg.norm(residual) / max(1., np.linalg.norm(Q), np.linalg.norm(A.T@P))
    poles = np.linalg.eigvals(A-B@K)
    if residual_rel > 1e-6 or np.max(poles.real) >= 0:
        raise ValueError(f"CARE 或闭环校验失败：相对残差={residual_rel:.3g}，最大极点实部={np.max(poles.real):.3g}")
    return K, P, residual_rel, poles


def main():
    parser = argparse.ArgumentParser(description="单侧偏心轮腿连续时间 LQR")
    parser.add_argument("--example", action="store_true", help="用仓库现有数据与方案中的示例权重演示；不是最终增益")
    parser.add_argument("--save", type=Path, help="可选：将本次参数和矩阵保存为 JSON")
    args = parser.parse_args()
    if args.example:
        p = Parameters()
        phi_e = 0.0
        q = np.array([20., 0.1, 80., 110., 700., 1.])
        r = np.array([90., 4.])
        print("示例计算：物理参数来自现有表/笔记；Q/R 来自方案.md 的起点，未作为实机权重确认。")
    else:
        p, phi_e, q, r = ask_inputs()

    state_e, input_e, A, B = linearize(p, phi_e)
    K, P, residual, poles = continuous_lqr(A, B, q, r)
    np.set_printoptions(precision=9, suppress=True, linewidth=140)
    print("\n状态顺序: [theta, theta_dot, x, x_dot, phi, phi_dot]")
    print("输入顺序: [T, T_p]")
    print("平衡状态 xi_e =", state_e)
    print("平衡输入 u_e =", input_e)
    print("平衡腿角 theta_e (deg) =", np.degrees(state_e[0]))
    print("\n连续 A =\n", A)
    print("\n连续 B =\n", B)
    print("\n连续 LQR 增益 K (2x6；第一行轮力矩、第二行髋力矩) =\n", K)
    print("\n控制律: u = u_e - K @ (xi - xi_e)")
    print("闭环极点 =", poles)
    print("CARE 相对残差 =", residual)
    if args.example:
        print("此 K 仅用于演示计算链路，请输入自己的全部参数和 Q/R 后重算。")

    if args.save:
        result = {
            "example_only": args.example, "parameter_convention": "single wheel + single leg + half body",
            "parameters_used": asdict(p), "phi_e_rad": phi_e,
            "Q_diag": q.tolist(), "R_diag": r.tolist(),
            "state_order": ["theta", "theta_dot", "x", "x_dot", "phi", "phi_dot"],
            "input_order": ["T", "T_p"], "xi_e": state_e.tolist(), "u_e": input_e.tolist(),
            "A": A.tolist(), "B": B.tolist(), "K": K.tolist(), "P": P.tolist(),
            "closed_loop_poles": [[float(z.real), float(z.imag)] for z in poles],
            "care_relative_residual": float(residual),
        }
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("已保存:", args.save)


if __name__ == "__main__":
    main()
