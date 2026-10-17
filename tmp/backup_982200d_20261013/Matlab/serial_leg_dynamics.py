"""Fixed-shape eccentric-leg model; SI units, state [theta,dtheta,x,dx,phi,dphi].

Single-side model: one wheel, one leg, half the body mass and inertia.
Ip is ONE leg's pitch-axis inertia about its own center of mass; it is not
halved. The parallel-axis term mp*(L^2+y^2) is added once in the mass matrix.
Ip, offset sign and leg length were confirmed by the user on 2026-10-10.
Body mass retains the supplied workbook value; assembly accounting is separate.
Run with Python + numpy. Outputs are written beside this script.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import numpy as np


@dataclass
class Parameters:
    R: float = 0.06
    Lt: float = 0.21248
    L: float = 0.134182
    y: float = -0.065062
    l: float = 0.043437
    mw: float = 0.135
    mp: float = 1.255
    M: float = 15.274 / 2
    Iw: float = 0.000388967
    Ip: float = 0.1466  # User-confirmed single-leg inertia about its COM.
    IM: float = 0.3277 / 2
    g: float = 9.81


def components(state, p):
    t, wt, _, _, f, wf = state
    a = p.L*np.sin(t) + p.y*np.cos(t)
    b = p.L*np.cos(t) - p.y*np.sin(t)
    h, z = p.Lt*np.sin(t), p.Lt*np.cos(t)
    H = np.array([
        [p.Ip+p.mp*(p.L**2+p.y**2)+p.M*p.Lt**2,
         p.mp*b+p.M*z, -p.M*p.Lt*p.l*np.cos(t+f)],
        [p.mp*b+p.M*z, p.mw+p.Iw/p.R**2+p.mp+p.M,
         -p.M*p.l*np.cos(f)],
        [-p.M*p.Lt*p.l*np.cos(t+f), -p.M*p.l*np.cos(f),
         p.IM+p.M*p.l**2]])
    r = np.array([
        p.g*(p.mp*a+p.M*h)-p.M*p.Lt*p.l*np.sin(t+f)*wf**2,
        (p.mp*a+p.M*h)*wt**2-p.M*p.l*np.sin(f)*wf**2,
        p.M*p.g*p.l*np.sin(f)-p.M*p.Lt*p.l*np.sin(t+f)*wt**2])
    D = np.array([[-1., 1.], [1/p.R, 0.], [0., 1.]])
    return H, r, D


def dynamics(state, control, p):
    H, r, D = components(state, p)
    acc = np.linalg.solve(H, r+D@control)
    return np.array([state[1], acc[0], state[3], acc[1], state[5], acc[2]])


def equilibrium(p, phi=0.):
    c = p.mp*p.L+p.M*p.Lt
    d = p.mp*p.y
    arg = p.M*p.l*np.sin(phi)/np.hypot(c, d)
    if abs(arg) > 1:
        raise ValueError('No static equilibrium for requested phi')
    theta = np.arcsin(arg)-np.arctan2(d, c)  # near-upright branch
    return np.array([theta, 0., 0., 0., phi, 0.]), np.array([0., -p.M*p.g*p.l*np.sin(phi)])


def linearize(p, phi=0.):
    state, control = equilibrium(p, phi)
    H, _, D = components(state, p)
    t = state[0]
    S = np.diag([p.g*(p.mp*(p.L*np.cos(t)-p.y*np.sin(t))
                           +p.M*p.Lt*np.cos(t)), 0., p.M*p.g*p.l*np.cos(phi)])
    F, G = np.linalg.solve(H, S), np.linalg.solve(H, D)
    A, B = np.zeros((6, 6)), np.zeros((6, 2))
    A[0, 1] = A[2, 3] = A[4, 5] = 1
    A[np.ix_([1, 3, 5], [0, 2, 4])] = F
    B[[1, 3, 5], :] = G
    return state, control, A, B


def newton_euler_residual(state, control, p):
    """Independent substitution into the eight force/moment equations below."""
    t, wt, _, _, f, wf = state
    at, ax, af = dynamics(state, control, p)[[1, 3, 5]]
    T, Tp = control
    a = p.L*np.sin(t)+p.y*np.cos(t)
    b = p.L*np.cos(t)-p.y*np.sin(t)
    h, z = p.Lt*np.sin(t), p.Lt*np.cos(t)
    NM = p.M*(ax+z*at-h*wt**2-p.l*np.cos(f)*af+p.l*np.sin(f)*wf**2)
    PM = p.M*(p.g-h*at-z*wt**2-p.l*np.sin(f)*af-p.l*np.cos(f)*wf**2)
    N = NM+p.mp*(ax+b*at-a*wt**2)
    P = PM+p.mp*(p.g-a*at-b*wt**2)
    Nf = p.mw*ax+N
    return np.array([
        p.mw*ax-(Nf-N), p.Iw*ax/p.R-(T-Nf*p.R),
        N-NM-p.mp*(ax+b*at-a*wt**2),
        P-PM-p.mp*p.g-p.mp*(-a*at-b*wt**2),
        p.Ip*at-(P*a+PM*(h-a)-N*b-NM*(z-b)-T+Tp),
        NM-p.M*(ax+z*at-h*wt**2-p.l*np.cos(f)*af+p.l*np.sin(f)*wf**2),
        PM-p.M*p.g-p.M*(-h*at-z*wt**2-p.l*np.sin(f)*af-p.l*np.cos(f)*wf**2),
        p.IM*af-(Tp+NM*p.l*np.cos(f)+PM*p.l*np.sin(f))])


def check(p):
    rng = np.random.default_rng(20261010)
    worst_residual = 0.
    for _ in range(100):
        state = rng.uniform(-0.5, 0.5, 6)
        control = rng.uniform(-1., 1., 2)
        H, _, _ = components(state, p)
        assert np.linalg.eigvalsh(H).min() > 0
        worst_residual = max(worst_residual, np.max(np.abs(newton_euler_residual(state, control, p))))
    worst_jacobian = 0.
    for phi in [0., 0.12]:
        state, control, A, B = linearize(p, phi)
        assert np.max(np.abs(dynamics(state, control, p))) < 1e-12
        eps = 1e-6
        An = np.column_stack([(dynamics(state+eps*e, control, p)-dynamics(state-eps*e, control, p))/(2*eps) for e in np.eye(6)])
        Bn = np.column_stack([(dynamics(state, control+eps*e, p)-dynamics(state, control-eps*e, p))/(2*eps) for e in np.eye(2)])
        worst_jacobian = max(worst_jacobian, np.max(np.abs(A-An)), np.max(np.abs(B-Bn)))
    assert worst_residual < 1e-10
    assert worst_jacobian < 1e-6
    return {'newton_euler_max_abs_residual': float(worst_residual),
            'finite_difference_max_abs_error': float(worst_jacobian)}


if __name__ == '__main__':
    p = Parameters()
    state, control, A, B = linearize(p)
    C = np.hstack([np.linalg.matrix_power(A, i)@B for i in range(6)])
    result = {'status': 'Single-side fixed-shape model; user-confirmed Ip, offset sign and Lt; body mass from workbook',
              'model_scope': 'one wheel + one leg + half body; equivalent five-bar transmission per user',
              'Ip_convention': 'one leg about its own center of mass; entered directly, not divided by 2; parallel-axis term added once',
              'mass_source_note': 'M=15.274/2 retained from workbook; half-body convention does not independently verify assembly mass subtraction',
              'parameters': asdict(p), 'state_equilibrium': state.tolist(),
              'input_equilibrium': control.tolist(), 'A': A.tolist(), 'B': B.tolist(),
              'checks': check(p), 'controllability_rank': int(np.linalg.matrix_rank(C))}
    # Zero offset must recover the conventional upright, collinear model.
    centered = Parameters(y=0.)
    assert abs(equilibrium(centered)[0][0]) < 1e-15
    result['checks']['zero_offset_equilibrium'] = 'passed'
    out = Path(__file__).with_name('serial_leg_dynamics_result.json')
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    np.set_printoptions(precision=9, suppress=True)
    print(result['status'])
    print('theta equilibrium (deg):', np.degrees(state[0]))
    print('A =\n', A)
    print('B =\n', B)
    print(result['checks'])
    print('controllability rank:', result['controllability_rank'])
