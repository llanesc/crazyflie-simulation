"""NumPy mecanum-drive dynamics for the Yahboom RosMaster X3.

Pure-NumPy port of crazyflie_rover_landing/envs/mecanum_dynamics.py (JAX).
Used inside CrazySim to step the rover kinematically.

State:   [x, y, c, s, vx, vy, wz]   (7D)
Control: [vx_cmd, vy_cmd, wz_cmd]    (3D)
"""

import numpy as np

# Physical constants (RosMaster X3)
WHEEL_RADIUS: float = 0.0325
HALF_WHEELBASE: float = 0.08
HALF_TRACK: float = 0.0845
_K: float = HALF_WHEELBASE + HALF_TRACK  # 0.1645 m

_TAU_MOTOR: float = 0.1  # s

WHEEL_VEL_MAX: float = 34.9  # rad/s
VX_CMD_MAX: float = 1.0  # m/s
VY_CMD_MAX: float = 1.0  # m/s
WZ_CMD_MAX: float = 5.0  # rad/s


def _inv_kinematics(vx: float, vy: float, wz: float) -> np.ndarray:
    r_inv = 1.0 / WHEEL_RADIUS
    return np.array([
        r_inv * (vx - vy - _K * wz),
        r_inv * (vx + vy + _K * wz),
        r_inv * (vx + vy - _K * wz),
        r_inv * (vx - vy + _K * wz),
    ])


def _fwd_kinematics(wheels: np.ndarray):
    r4 = WHEEL_RADIUS / 4.0
    vx = r4 * (wheels[0] + wheels[1] + wheels[2] + wheels[3])
    vy = r4 * (-wheels[0] + wheels[1] + wheels[2] - wheels[3])
    wz = r4 / _K * (-wheels[0] + wheels[1] - wheels[2] + wheels[3])
    return vx, vy, wz


def _ode(x: np.ndarray, u: np.ndarray, wheel_vel_max: float) -> np.ndarray:
    c, s = x[2], x[3]
    vx, vy, wz = x[4], x[5], x[6]

    wheels_target = _inv_kinematics(u[0], u[1], u[2])
    wheels_target = np.clip(wheels_target, -wheel_vel_max, wheel_vel_max)

    wheels_current = _inv_kinematics(vx, vy, wz)
    wheels_dot = (wheels_target - wheels_current) / _TAU_MOTOR

    dvx, dvy, dwz = _fwd_kinematics(wheels_dot)

    dx = vx * c - vy * s
    dy = vx * s + vy * c
    dc = -wz * s
    ds = wz * c

    return np.array([dx, dy, dc, ds, dvx, dvy, dwz])


def mecanum_step(
    state: np.ndarray,
    control: np.ndarray,
    dt: float,
    wheel_vel_max: float = WHEEL_VEL_MAX,
) -> np.ndarray:
    """RK4-integrate the mecanum model for one timestep."""
    u = np.array([
        np.clip(control[0], -VX_CMD_MAX, VX_CMD_MAX),
        np.clip(control[1], -VY_CMD_MAX, VY_CMD_MAX),
        np.clip(control[2], -WZ_CMD_MAX, WZ_CMD_MAX),
    ])

    k1 = _ode(state, u, wheel_vel_max)
    k2 = _ode(state + dt / 2 * k1, u, wheel_vel_max)
    k3 = _ode(state + dt / 2 * k2, u, wheel_vel_max)
    k4 = _ode(state + dt * k3, u, wheel_vel_max)
    nxt = state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    # Re-normalize (c, s)
    norm = np.sqrt(nxt[2] ** 2 + nxt[3] ** 2)
    nxt[2] /= norm
    nxt[3] /= norm
    return nxt
