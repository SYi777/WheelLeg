"""Audit the configured K against the existing single-side dynamics.

Run with Python + numpy + scipy. No CAN, flash or hardware access.
This checks the symmetric fixed-shape model. Wheel differential power below
is an algebraic feedback-direction check, not a complete yaw dynamics model.
"""
from dataclasses import asdict
import json
from pathlib import Path
import re

import numpy as np
from scipy.linalg import expm, solve_continuous_are

from serial_leg_dynamics import Parameters, check, linearize


def audit():
    root = Path(__file__).resolve().parent.parent
    source = (root / "Algorithm" / "balance.c").read_text(encoding="gbk")
    block = re.search(r"static const float K\[2\]\[6\]\s*=\s*\{(.*?)\};", source, re.S)
    if block is None:
        raise ValueError("Cannot locate the configured K")
    K = np.asarray([float(x) for x in re.findall(r"[-+]?\d+\.\d+(?:[eE][-+]?\d+)?", block[1])]).reshape(2, 6)
    # Parameters from the user's CLI calculation; Ip is about ONE leg's COM.
    p = Parameters(y=-0.065042)
    state, control, A, B = linearize(p)
    Q, R = np.diag([10., 1., 50., 10., 800., 1.]), np.diag([20., 1.])
    P = solve_continuous_are(A, B, Q, R)
    computed = np.linalg.solve(R, B.T @ P)
    poles = np.linalg.eigvals(A - B @ K)
    combined = np.zeros((8, 8))
    combined[:6, :6], combined[:6, 6:] = A, B
    zoh = expm(combined * 0.001)
    spectral_radius = np.max(np.abs(np.linalg.eigvals(zoh[:6, :6] - zoh[:6, 6:] @ K)))
    # Same mean state, velocities differ by 0.1 m/s. This is the velocity-only
    # differential contribution: old independent LQR injects differential power.
    velocity = np.array([-0.05, 0.05])
    old_torque = -K[0, 3] * velocity
    delta = velocity[1] - velocity[0]
    new_torque = np.array([delta, -delta])  # kd=1 N*m/(m/s)
    old_power, new_power = old_torque @ velocity / p.R, new_torque @ velocity / p.R
    assert np.max(poles.real) < 0 and spectral_radius < 1
    assert np.max(np.abs(K - computed)) < 1e-3
    assert old_power > 0 and new_power < 0
    result = {
        "scope": "Symmetric fixed-shape, nonsaturated, no-slip model; not a physical standing validation",
        "parameters": asdict(p), "Q_diagonal": list(np.diag(Q)), "R_diagonal": list(np.diag(R)),
        "K_configured": K.tolist(), "K_recomputed": computed.tolist(),
        "K_max_abs_difference": float(np.max(np.abs(K - computed))),
        "model_equilibrium": state.tolist(), "model_input_equilibrium": control.tolist(),
        "continuous_poles_real_imag": [[float(z.real), float(z.imag)] for z in poles],
        "sample_time_s": 0.001, "sampled_closed_loop_spectral_radius": float(spectral_radius),
        "velocity_only_differential_example": {
            "v_L_v_R_m_s": velocity.tolist(),
            "old_model_T_L_T_R_Nm": old_torque.tolist(), "old_power_W": float(old_power),
            "new_model_T_L_T_R_Nm": new_torque.tolist(), "new_power_W": float(new_power),
            "note": "Only differential wheel power; neither calculation proves full yaw-mode stability",
        },
        "dynamics_checks": check(p),
    }
    output = root / "Matlab" / "control_feedback_audit_result.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print("PASS: K/CARE, continuous stability, 1 ms ZOH stability, differential damping direction, dynamics residuals.")
    print(f"K max difference: {result['K_max_abs_difference']:.3g}")
    print(f"Sampled spectral radius: {spectral_radius:.12f}")
    print(f"Velocity-only differential power: old={old_power:.6f} W; new={new_power:.6f} W")
    print(output)
    return result


if __name__ == "__main__":
    audit()
