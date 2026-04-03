#!/usr/bin/env python3
"""
CrazySim MuJoCo — SITL interface for crazyflie-firmware (multi-agent)
Mirrors the Gazebo CrazySim plugin protocol exactly.

Protocol (socketlink.c / CrtpUtils.h):
  Handshake:
    Firmware → sim : 0xF3 (1 byte)
    Sim → firmware : 0xF3 (1 byte)

  IMU packet (sim → firmware, CRTP port 0x09 ch 0):
    byte 0    : header = 0x90
    byte 1    : type   = SENSOR_GYRO_ACC_SIM (0)
    bytes 2-7 : Axis3i16 acc  (3 × int16, little-endian)
    bytes 8-13: Axis3i16 gyro (3 × int16, little-endian)

  Baro packet (sim → firmware, CRTP port 0x09 ch 0):
    byte 0    : header = 0x90
    byte 1    : type   = SENSOR_BARO_SIM (2)
    bytes 2-5 : float  pressure   [mbar]
    bytes 6-9 : float  temperature [°C]
    bytes 10-13: float asl        [m]

  Range packet (sim → firmware, CRTP port 0x09 ch 0):
    byte 0    : header = 0x90
    byte 1    : type   = SENSOR_RANGE_SIM (3)
    bytes 2-21: 5 × float32 (front, back, left, right, up) [m]

  Pose packet (sim → firmware, CRTP port 0x06 ch 1):
    byte 0    : header = 0x65
    byte 1    : id     = CRTP_GEN_LOC_ID_EXT_POS (0x08)
    bytes 2-29: x,y,z,qx,qy,qz,qw as float32

  Motor packet (firmware → sim, CRTP port 0x09 ch 0):
    byte 0    : header = 0x90
    bytes 1-8 : 4 × uint16 motor PWM [0..65535]

Supported model types (--model-type):
  cf2x_L250   Loco-250 props   thrust_max=0.12 N/motor  (drone-models params)
  cf2x_P250   Pixy-250 props   thrust_max=0.12 N/motor  (drone-models params)
  cf2x_T350   Thrust-350 props thrust_max=0.18 N/motor  (Gazebo CrazySim values)
  cf21B_500   21B body/500     thrust_max=0.20 N/motor  (drone-models params)

Multi-agent usage:
  # Start N firmware instances first:
  #   ./cf2 19950 &
  #   ./cf2 19951 &
  #   ...

  # Single agent (backward compatible):
  python3 crazysim.py --vis

  # Multiple agents with spawn positions:
  python3 crazysim.py --vis --agents 0,0 1,0 0,1
  python3 crazysim.py --vis --spawn-file drone_spawn_list/two_example.txt
"""

import argparse
import math
import os
import queue
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import mujoco
import mujoco.viewer
import numpy as np

# ---------------------------------------------------------------------------
# CRTP constants (CrtpUtils.h)
# ---------------------------------------------------------------------------
CRTP_PORT_SIM    = 0x09
CRTP_PORT_LOC    = 0x06
CRTP_HDR_SIM     = (CRTP_PORT_SIM << 4) | 0           # 0x90  (SIM port, ch 0 — motors)
CRTP_HDR_LED     = (CRTP_PORT_SIM << 4) | 1           # 0x91  (SIM port, ch 1 — LED RGB)
CRTP_HDR_HEADLIGHT = (CRTP_PORT_SIM << 4) | 2         # 0x92  (SIM port, ch 2 — headlight)
CRTP_HDR_LOC     = (CRTP_PORT_LOC << 4) | (1 << 2) | 1  # 0x65

SENSOR_GYRO_ACC  = 0
SENSOR_BARO      = 2
SENSOR_RANGE     = 3
SENSOR_TOF       = 5
SENSOR_FLOW      = 6
GEN_LOC_EXT_POSE = 0x08

# LSB conversion factors (firmware sensors_sitl.c)
SENSORS_G_PER_LSB   = (2.0 * 16.0)   / 65536.0   # G / LSB
SENSORS_DEG_PER_LSB = (2.0 * 2000.0) / 65536.0   # deg/s / LSB
GRAVITY             = 9.81
DEG_TO_RAD          = math.pi / 180.0

# Drag-torque reaction on body per motor:
#   CCW prop (motor0,2) → body receives -Z torque (reaction to CCW spin)
#   CW  prop (motor1,3) → body receives +Z torque (reaction to CW  spin)
MOTOR_DIR = np.array([-1.0, 1.0, -1.0, 1.0])

# Standard atmosphere for baro simulation
T0_K     = 288.15
P0_PA    = 101325.0
L_LAPSE  = 0.0065
R_GAS    = 8.314
M_AIR    = 0.0289644
BARO_RATE_HZ = 50
RANGE_RATE_HZ = 10         # Multi-ranger update rate (matches real hardware)
RANGE_MAX_M   = 4.0        # VL53L1x max range [m]
FLOW_RATE_HZ  = 100        # PMW3901 flow sensor update rate
TOF_RATE_HZ   = 40         # VL53L1x ToF update rate (flowdeck)
FLOW_NPIX     = 30.0       # PMW3901 pixel count
FLOW_THETAPIX = 4.2 * math.pi / 180.0  # FOV per pixel [rad]
CFLIB_PORT_OFFSET = -100   # cflib port = firmware port + offset (19950 → 19850)

# ---------------------------------------------------------------------------
# Per-model motor parameters — loaded from drone-models submodule params.toml
#
# Thrust/torque model (crazyflow polynomial, RPM-based):
#   thrust [N]  = rpm2thrust[0] + rpm2thrust[1]*rpm + rpm2thrust[2]*rpm²
#   torque [Nm] = rpm2torque[0] + rpm2torque[1]*rpm + rpm2torque[2]*rpm²
#
# tau: 1 / rotor_dyn_coef_simple  [s]
# pwm_thrust_full: thrust_max     [N/motor]
# ---------------------------------------------------------------------------
try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib  # type: ignore[no-redef]

RPM_TO_RADS = 2.0 * math.pi / 60.0
RADS_TO_RPM = 60.0 / (2.0 * math.pi)


@dataclass(frozen=True)
class MotorParams:
    rpm2thrust:      tuple[float, float, float]
    rpm2torque:      tuple[float, float, float]
    tau_up:          float   # s
    tau_down:        float   # s
    pwm_thrust_full: float   # N
    max_rpm:         float   # RPM clamp
    mass:            float   # kg
    diaginertia:     tuple[float, float, float]  # Ixx, Iyy, Izz [kg·m²]
    drag_matrix:     np.ndarray  # 3×3 aerodynamic drag matrix [N·s/m]
    prop_inertia:    float       # single propeller inertia [kg·m²]
    prop_radius:     float       # propeller radius [m]


def _max_rpm(rpm2thrust: tuple[float, float, float], thrust_max: float) -> float:
    """Solve rpm2thrust polynomial for the RPM that gives thrust_max."""
    a, b, c = rpm2thrust[0], rpm2thrust[1], rpm2thrust[2]
    disc = b * b - 4.0 * c * (a - thrust_max)
    return (-b + math.sqrt(max(disc, 0.0))) / (2.0 * c)


def _load_motor_params() -> dict[str, MotorParams]:
    """Load motor parameters from the drone-models submodule params.toml."""
    params_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'drone-models', 'drone_models', 'data', 'params.toml')
    with open(params_path, 'rb') as f:
        all_params = tomllib.load(f)
    result = {}
    for name, p in all_params.items():
        rpm2thrust = tuple(p['rpm2thrust'])
        tau = 1.0 / p['rotor_dyn_coef_simple']
        J = p['J']
        drag_matrix = np.array(p['drag_matrix'], dtype=np.float64)
        prop_inertia = float(p['prop_inertia'])
        result[name] = MotorParams(
            rpm2thrust=rpm2thrust,
            rpm2torque=tuple(p['rpm2torque']),
            tau_up=tau,
            tau_down=tau,
            pwm_thrust_full=p['thrust_max'],
            max_rpm=_max_rpm(rpm2thrust, p['thrust_max']),
            mass=p['mass'],
            diaginertia=(J[0][0], J[1][1], J[2][2]),
            drag_matrix=drag_matrix,
            prop_inertia=prop_inertia,
            prop_radius=float(p['prop_radius']),
        )
    return result


MOTOR_PARAMS: dict[str, MotorParams] = _load_motor_params()

# Default MJCF paths per model type (relative to this script).
# Models come from the drone-models submodule (utiasDSL/drone-models).
_DRONE_MODELS_DATA = 'drone-models/drone_models/data'
MODEL_PATHS: dict[str, str] = {
    name: f'{_DRONE_MODELS_DATA}/{name}.xml' for name in MOTOR_PARAMS
}

DEFAULT_MODEL_TYPE = 'cf2x_T350'


def _infer_model_type(model_path: str) -> str | None:
    """Guess model type from the MJCF filename, returns None if unknown."""
    stem = os.path.splitext(os.path.basename(model_path))[0]
    for key in MOTOR_PARAMS:
        if key in stem:
            return key
    return None


# ---------------------------------------------------------------------------
# Packet helpers
# ---------------------------------------------------------------------------

def acc_to_lsb(a: float) -> int:
    """Accelerometer [m/s²] → int16 LSB."""
    return int(np.clip(a / (SENSORS_G_PER_LSB * GRAVITY), -32768, 32767))


def gyro_to_lsb(w: float) -> int:
    """Gyro [rad/s] → int16 LSB."""
    return int(np.clip(w / (SENSORS_DEG_PER_LSB * DEG_TO_RAD), -32768, 32767))


def alt_to_pressure_mbar(alt_m: float) -> float:
    """Standard atmosphere altitude [m] → pressure [mbar]."""
    p = P0_PA * (1.0 - L_LAPSE * alt_m / T0_K) ** (GRAVITY * M_AIR / (R_GAS * L_LAPSE))
    return p / 100.0


def make_imu_packet(acc: np.ndarray, gyro: np.ndarray) -> bytes:
    """Pack struct imu_s CRTP packet (14 bytes total)."""
    payload = struct.pack('<Bhhhhhh',
                          SENSOR_GYRO_ACC,
                          acc_to_lsb(acc[0]),  acc_to_lsb(acc[1]),  acc_to_lsb(acc[2]),
                          gyro_to_lsb(gyro[0]), gyro_to_lsb(gyro[1]), gyro_to_lsb(gyro[2]))
    return bytes([CRTP_HDR_SIM]) + payload   # 1 + 13 = 14 bytes


def make_baro_packet(alt_m: float, temp_c: float = 25.0) -> bytes:
    """Pack struct baro_s CRTP packet (14 bytes total)."""
    payload = struct.pack('<Bfff',
                          SENSOR_BARO,
                          alt_to_pressure_mbar(alt_m),
                          temp_c,
                          alt_m)
    return bytes([CRTP_HDR_SIM]) + payload   # 1 + 13 = 14 bytes


def make_pose_packet(pos: np.ndarray, quat_xyzw: np.ndarray) -> bytes:
    """Pack CrtpExtPose_s CRTP packet (30 bytes total)."""
    payload = struct.pack('<Bfffffff',
                          GEN_LOC_EXT_POSE,
                          pos[0], pos[1], pos[2],
                          quat_xyzw[0], quat_xyzw[1],
                          quat_xyzw[2], quat_xyzw[3])
    return bytes([CRTP_HDR_LOC]) + payload   # 1 + 29 = 30 bytes


def make_range_packet(front: float, back: float, left: float,
                      right: float, up: float) -> bytes:
    """Pack multi-ranger CRTP packet (22 bytes total)."""
    payload = struct.pack('<Bfffff', SENSOR_RANGE, front, back, left, right, up)
    return bytes([CRTP_HDR_SIM]) + payload   # 1 + 21 = 22 bytes


def make_tof_packet(distance: float) -> bytes:
    """Pack TOF (downward rangefinder) CRTP packet."""
    payload = struct.pack('<Bf', SENSOR_TOF, distance)
    return bytes([CRTP_HDR_SIM]) + payload   # 1 + 5 = 6 bytes


def make_flow_packet(dpixelx: float, dpixely: float, dt: float) -> bytes:
    """Pack optical flow CRTP packet (dpixelx, dpixely, dt)."""
    payload = struct.pack('<Bfff', SENSOR_FLOW, dpixelx, dpixely, dt)
    return bytes([CRTP_HDR_SIM]) + payload   # 1 + 13 = 14 bytes


# ---------------------------------------------------------------------------
# Sensor Noise Model
# ---------------------------------------------------------------------------
# BMI088 datasheet (BST-BMI088-DS001) noise, bias, and scale specs.
# Bias and scale are randomized per instance to simulate unit-to-unit variation.
_BMI088 = {
    # White noise density
    'acc_noise_density': [0.00157, 0.00157, 0.00186],  # m/s²/√Hz [X,Y,Z] (160,160,190 µg/√Hz)
    'gyro_noise_density': 0.000244,                     # rad/s/√Hz (0.014 °/s/√Hz)
    # Bias random walk (Allan variance, 5min static, real CF2.1)
    'acc_bias_walk': 0.000001,     # m/s²/√s
    'gyro_bias_walk': 0.0000013,   # rad/s/√s
    # Zero offset ranges (datasheet)
    'acc_offset_mg': 20.0,         # ±20 mg zero-g offset
    'gyro_offset_dps': 1.0,        # ±1 °/s zero-rate offset
    # Scale/sensitivity tolerance (datasheet)
    'gyro_scale_pct': 1.0,         # ±1% sensitivity tolerance
    # Barometer
    'baro_noise_std': 0.1,         # m
}


class SensorNoiseModel:
    """BMI088 sensor model: output = scale * true + bias + noise + walk.

    Bias and scale are randomized at construction to simulate unit-to-unit
    variation within datasheet tolerances.
    """

    def __init__(self, dt: float = 0.001):
        cfg = _BMI088
        # Noise density (per-axis)
        acc_nd = cfg['acc_noise_density']
        self._acc_nd = np.array(acc_nd) if isinstance(acc_nd, list) else np.full(3, acc_nd)
        gyro_nd = cfg['gyro_noise_density']
        self._gyro_nd = np.array(gyro_nd) if isinstance(gyro_nd, list) else np.full(3, gyro_nd)
        # Bias random walk
        self._acc_bw = cfg['acc_bias_walk']
        self._gyro_bw = cfg['gyro_bias_walk']
        self._baro_std = cfg['baro_noise_std']
        self._dt = dt
        self._sqrt_dt = math.sqrt(dt)
        # Fixed bias — randomized at startup from datasheet offset range
        acc_bias_max = cfg['acc_offset_mg'] * 1e-3 * 9.81  # mg → m/s²
        gyro_bias_max = cfg['gyro_offset_dps'] * math.pi / 180.0  # °/s → rad/s
        self._acc_bias_fixed = np.random.uniform(-acc_bias_max, acc_bias_max, 3)
        self._gyro_bias_fixed = np.random.uniform(-gyro_bias_max, gyro_bias_max, 3)
        # Scale factor — randomized at startup from datasheet tolerance
        gyro_scale_err = cfg['gyro_scale_pct'] / 100.0
        self._acc_scale = np.ones(3)  # accel scale tolerance negligible (0.002%/K)
        self._gyro_scale = 1.0 + np.random.uniform(-gyro_scale_err, gyro_scale_err, 3)
        # Wandering bias (starts at zero, drifts over time)
        self._acc_bias_walk = np.zeros(3)
        self._gyro_bias_walk = np.zeros(3)

        print(f'[sensor] acc  bias: [{self._acc_bias_fixed[0]:+.4f}, '
              f'{self._acc_bias_fixed[1]:+.4f}, {self._acc_bias_fixed[2]:+.4f}] m/s²')
        print(f'[sensor] gyro bias: [{self._gyro_bias_fixed[0]*180/math.pi:+.4f}, '
              f'{self._gyro_bias_fixed[1]*180/math.pi:+.4f}, '
              f'{self._gyro_bias_fixed[2]*180/math.pi:+.4f}] °/s')
        print(f'[sensor] gyro scale: [{self._gyro_scale[0]:.4f}, '
              f'{self._gyro_scale[1]:.4f}, {self._gyro_scale[2]:.4f}]')

    def apply_imu_noise(self, acc: np.ndarray, gyro: np.ndarray) -> tuple:
        """Apply sensor model: output = scale * true + bias + noise + walk."""
        # Bias walk
        self._acc_bias_walk += np.random.normal(0, self._acc_bw * self._sqrt_dt, 3)
        self._gyro_bias_walk += np.random.normal(0, self._gyro_bw * self._sqrt_dt, 3)
        # White noise
        acc_noise = np.random.normal(0, 1, 3) * self._acc_nd * self._sqrt_dt
        gyro_noise = np.random.normal(0, 1, 3) * self._gyro_nd * self._sqrt_dt
        # Full model
        acc_out = self._acc_scale * acc + self._acc_bias_fixed + self._acc_bias_walk + acc_noise
        gyro_out = self._gyro_scale * gyro + self._gyro_bias_fixed + self._gyro_bias_walk + gyro_noise
        return acc_out, gyro_out

    def apply_baro_noise(self, alt_m: float) -> float:
        """Add Gaussian noise to barometer altitude reading."""
        return alt_m + np.random.normal(0, self._baro_std)


# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Wind & Turbulence Model
# ---------------------------------------------------------------------------
# Dryden turbulence intensity presets (sigma_w at low altitude) [m/s]
_TURBULENCE_SIGMA = {'none': 0.0, 'light': 0.5, 'moderate': 1.5, 'severe': 3.0}


class WindModel:
    """Constant wind + Ornstein-Uhlenbeck gusts + Dryden turbulence."""

    def __init__(self, speed: float = 0.0, direction_deg: float = 0.0,
                 gust_intensity: float = 0.0, turbulence: str = 'none',
                 dt: float = 0.001):
        rad = math.radians(direction_deg)
        self._const_wind = np.array([speed * math.cos(rad),
                                     speed * math.sin(rad), 0.0])
        self._gust_intensity = gust_intensity
        self._gust_tau = 4.0  # correlation time [s]
        self._gust_state = np.zeros(3)
        self._turb_sigma = _TURBULENCE_SIGMA.get(turbulence, 0.0)
        self._turb_state = np.zeros(3)
        self._turb_tau = 5.0  # Dryden length-scale / airspeed proxy [s]
        self._dt = dt
        self._sqrt_dt = math.sqrt(dt)

    def get_wind_velocity(self, pos: np.ndarray, t: float) -> np.ndarray:
        """Return 3D wind velocity in world frame at (pos, t)."""
        wind = self._const_wind.copy()
        # Ornstein-Uhlenbeck gust process
        if self._gust_intensity > 0:
            alpha = self._dt / self._gust_tau
            self._gust_state *= (1.0 - alpha)
            self._gust_state += math.sqrt(2.0 * alpha) * self._gust_intensity * \
                np.random.normal(0, 1, 3)
            wind += self._gust_state
        # Dryden turbulence (simplified first-order shaping filter)
        if self._turb_sigma > 0:
            alpha = self._dt / self._turb_tau
            self._turb_state *= (1.0 - alpha)
            self._turb_state += math.sqrt(2.0 * alpha) * self._turb_sigma * \
                np.random.normal(0, 1, 3)
            wind += self._turb_state
        return wind


# ---------------------------------------------------------------------------
# Camera Renderer — pushes raw frames to crazysim_cpx.py via UDP
# ---------------------------------------------------------------------------
CAM_FRAME_BASE_PORT = 5200  # internal frame-push port (agent N uses +N)
CAM_FRAME_CHUNK = 60000     # UDP payload limit (~64KB, stay under)


class CameraRenderer:
    """Offscreen FPV camera that pushes raw grayscale frames via UDP
    to ``crazysim_cpx.py``, which wraps them in CPX and serves to cflib.

    Frames are split into numbered UDP chunks so they fit in datagrams.
    Chunk format: [seq:u16][total:u16][width:u16][height:u16][data]
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 agent_id: int, width: int = 324, height: int = 244,
                 fps: float = 20.0, cam_port: int | None = None):
        self._cam_name = f'cf{agent_id}_fpv_cam'
        self._cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA,
                                          self._cam_name)
        if self._cam_id < 0:
            raise ValueError(f'Camera {self._cam_name!r} not found in model')
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self._width = width
        self._height = height
        self._frame_period = 1.0 / fps
        self._acc = 0.0

        self._port = cam_port if cam_port is not None else CAM_FRAME_BASE_PORT + agent_id
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dest = ('127.0.0.1', self._port)
        print(f'[crazysim] Agent {agent_id}: camera frames → '
              f'udp://127.0.0.1:{self._port}')

    def maybe_render(self, model: mujoco.MjModel, data: mujoco.MjData,
                     dt: float):
        self._acc += dt
        if self._acc < self._frame_period:
            return
        self._acc -= self._frame_period

        self._renderer.update_scene(data, camera=self._cam_name)
        pixels = self._renderer.render()
        gray = np.dot(pixels[..., :3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)
        img_bytes = gray.tobytes()

        # Split into UDP chunks
        total = (len(img_bytes) + CAM_FRAME_CHUNK - 1) // CAM_FRAME_CHUNK
        for seq in range(total):
            offset = seq * CAM_FRAME_CHUNK
            chunk = img_bytes[offset:offset + CAM_FRAME_CHUNK]
            header = struct.pack('<HHHH', seq, total, self._width, self._height)
            try:
                self._sock.sendto(header + chunk, self._dest)
            except OSError:
                pass

    def stop(self):
        self._sock.close()


# ---------------------------------------------------------------------------
# Directory containing this script — used to resolve scene.xml
_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_XML = os.path.join(_HERE, 'scene.xml')


def _patch_drone_spec(spec: mujoco.MjSpec, params: MotorParams) -> None:
    """Patch an upstream drone-models spec with CrazySim requirements.

    - Override mass and inertia from params.toml (authoritative source)
    - Add IMU site and accelerometer/gyro sensors
    - Add LED emissive materials, headlight geometry, and point lights
    - Fix mesh orientation (euler 0 0 90 aligns STL front with body +X)
    """
    drone = spec.body('drone')
    # Override mass and inertia from params.toml
    drone.mass = params.mass
    drone.inertia = [params.diaginertia[0], params.diaginertia[1], params.diaginertia[2]]
    # Swap collision geometry: disable sphere, enable box
    col_sphere = spec.geom('col_sphere')
    if col_sphere is not None:
        col_sphere.contype = 0
        col_sphere.conaffinity = 0
    col_box = spec.geom('col_box')
    if col_box is not None:
        col_box.contype = 1
        col_box.conaffinity = 1
    # Add IMU site at body center (if not already present)
    if spec.site('imu') is None:
        drone.add_site(name='imu', pos=[0, 0, 0], group=5)
    # Add accelerometer + gyro sensors (if not already present)
    if not any(s.name == 'acc' for s in spec.sensors):
        spec.add_sensor(name='acc', type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    if not any(s.name == 'gyro' for s in spec.sensors):
        spec.add_sensor(name='gyro', type=mujoco.mjtSensor.mjSENS_GYRO,
                        objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')

    # --- LED and headlight setup ---

    # Make LED materials emissive so they glow in dark scenes
    for mat_name in ('led_top', 'led_bot'):
        mat = spec.material(mat_name)
        if mat is not None:
            mat.emission = 1.0

    # Add headlight material (emissive, initially transparent)
    if spec.material('headlight') is None:
        spec.add_material(name='headlight', rgba=[1.0, 1.0, 0.95, 0.0], emission=1.0)

    # Add headlight geometry at front of PCB
    if spec.geom('headlight') is None:
        g = drone.add_geom(name='headlight')
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [0.002, 0.004, 0.002]
        g.pos = [0.033, 0, 0.003]
        g.material = 'headlight'
        g.contype = 0
        g.conaffinity = 0
        g.group = 2

    # Fix mesh orientations: compose Z-90° rotation to align STL front with body +X.
    _COS45 = 0.7071067811865476
    # quat for Z-90°: [cos(-45°), 0, 0, sin(-45°)] = [cos45, 0, 0, -sin45]
    qz = np.array([_COS45, 0.0, 0.0, -_COS45])
    for geom in drone.geoms:
        if geom.name in ('col_sphere', 'col_box', 'headlight'):
            continue
        if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
            q0 = np.array(geom.quat)
            w0, x0, y0, z0 = q0
            w1, x1, y1, z1 = qz
            geom.quat = [
                w1*w0 - x1*x0 - y1*y0 - z1*z0,
                w1*x0 + x1*w0 + y1*z0 - z1*y0,
                w1*y0 - x1*z0 + y1*w0 + z1*x0,
                w1*z0 + x1*y0 - y1*x0 + z1*w0,
            ]

    # Add omnidirectional point lights at body center
    for light_name in ('led_top_light', 'led_bot_light'):
        if not any(l.name == light_name for l in drone.lights):
            light = drone.add_light(name=light_name)
            light.pos = [0, 0, 0]
            light.diffuse = [0, 0, 0]
            light.specular = [0, 0, 0]
            light.attenuation = [0, 0, 5]
            light.cutoff = 180
            light.exponent = 0
            light.castshadow = False
            light.active = False

    # Add directional headlight spot
    if not any(l.name == 'headlight_light' for l in drone.lights):
        light = drone.add_light(name='headlight_light')
        light.pos = [0, 0, 0]
        light.dir = [1, 0, 0]
        light.diffuse = [0, 0, 0]
        light.specular = [0, 0, 0]
        light.attenuation = [0, 0, 5]
        light.cutoff = 80
        light.exponent = 5
        light.castshadow = False
        light.active = False


def _build_spec_multi(drone_xml: str, spawn_positions: list[tuple[float, float]],
                      params: MotorParams) -> mujoco.MjSpec:
    """
    Combine scene.xml + N drone models using MjSpec.
    Each drone body is attached at a unique spawn position with a unique
    name prefix (cf0_, cf1_, ...) to avoid name collisions.
    """
    scene_spec = mujoco.MjSpec.from_file(_SCENE_XML)
    scene_spec.copy_during_attach = True

    for i, (x, y) in enumerate(spawn_positions):
        drone_spec = mujoco.MjSpec.from_file(drone_xml)
        _patch_drone_spec(drone_spec, params)
        frame = scene_spec.worldbody.add_frame()
        frame.pos = [x, y, 0.0]
        prefix = f'cf{i}_'
        attached = frame.attach_body(drone_spec.body('drone'), prefix, '')
        attached.add_freejoint()

    return scene_spec


def _build_spec_multi_safe(drone_xml: str, spawn_positions: list[tuple[float, float]],
                           params: MotorParams) -> mujoco.MjModel:
    """
    Build multi-agent model. Falls back to mesh-stripped drone if STL assets
    are missing.
    """
    try:
        return _build_spec_multi(drone_xml, spawn_positions, params).compile()
    except ValueError as exc:
        err = str(exc)
        if 'opening file' not in err and '.stl' not in err.lower():
            raise
        print('[crazysim] Mesh assets not found — loading without visual meshes.')
        print('[crazysim] (Run `git submodule update --init` to fetch drone-models.)')
        stripped_xml = _strip_meshes(drone_xml)

        scene_spec = mujoco.MjSpec.from_file(_SCENE_XML)
        scene_spec.copy_during_attach = True

        for i, (x, y) in enumerate(spawn_positions):
            drone_spec = mujoco.MjSpec.from_string(stripped_xml)
            _patch_drone_spec(drone_spec, params)
            frame = scene_spec.worldbody.add_frame()
            frame.pos = [x, y, 0.0]
            prefix = f'cf{i}_'
            attached = frame.attach_body(drone_spec.body('drone'), prefix, '')
            attached.add_freejoint()

        return scene_spec.compile()


def _strip_meshes(xml_path: str) -> str:
    """
    Parse the MJCF XML and return a string with all mesh-dependent elements
    removed: <mesh> assets, visual geoms (class="visual" or mesh= attribute),
    and <material> entries.  Leaves collision geometry and all functional
    elements (joints, sites, actuators, sensors) intact.
    """
    # Register default namespace so output is not mangled
    ET.register_namespace('', '')
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Remove <mesh> and <material> from <asset>
    for asset in root.findall('asset'):
        for child in list(asset):
            if child.tag in ('mesh', 'material'):
                asset.remove(child)
        if len(asset) == 0:
            root.remove(asset)

    # Remove visual geoms from every body (class="visual" or has mesh= attr)
    for body in root.iter('body'):
        for geom in list(body.findall('geom')):
            cls = geom.get('class', '')
            if 'visual' in cls or geom.get('mesh') is not None:
                body.remove(geom)

    # Remove <default class="visual"> blocks
    for default in root.iter('default'):
        for vis in list(default.findall("default[@class='visual']")):
            default.remove(vis)

    return ET.tostring(root, encoding='unicode')


# ---------------------------------------------------------------------------
class DroneAgent:
    """Per-drone state: UDP sockets, motor dynamics, sensor indices."""

    def __init__(self, agent_id: int, model: mujoco.MjModel, data: mujoco.MjData,
                 params: MotorParams, host: str, fw_port: int,
                 cflib_port: int, dt: float, *,
                 noise_model: SensorNoiseModel | None = None,
                 wind_model: WindModel | None = None,
                 ground_effect: bool = False,
                 camera_renderer: CameraRenderer | None = None,
                 flowdeck: bool = False):
        self.agent_id = agent_id
        self.model = model
        self.data = data
        self.dt = dt
        self._params = params
        prefix = f'cf{agent_id}_'

        # Body ID and free joint qvel address
        self._body_id = model.body(f'{prefix}drone').id
        jnt_id = model.body(f'{prefix}drone').jntadr[0]
        self._qvel_adr = model.jnt_dofadr[jnt_id]  # 6 DOFs: [vx,vy,vz,wx,wy,wz]

        # Sensor addresses
        self._acc_adr = model.sensor_adr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f'{prefix}acc')]
        self._gyro_adr = model.sensor_adr[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f'{prefix}gyro')]

        # Actuator indices
        self._act_force = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{prefix}motor{i}_force')
            for i in range(4)]
        self._act_torque = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{prefix}motor{i}_torque')
            for i in range(4)]

        # Motor state (tracked in RPM to match drone-models polynomial coefficients)
        self._rpm = np.zeros(4)
        self._rpm_ref = np.zeros(4)
        self._rpm_dot = np.zeros(4)  # RPM/s for gyroscopic z-torque
        self._motor_lock = threading.Lock()

        # LED material indices (for dynamic color updates from firmware)
        # Index 0 = bottom, index 1 = top
        self._led_bot_mat_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_MATERIAL, f'{prefix}led_bot')
        self._led_top_mat_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_MATERIAL, f'{prefix}led_top')
        self._led_bot_rgb = np.zeros(3)  # bottom LED color [0..1]
        self._led_top_rgb = np.zeros(3)  # top LED color [0..1]
        self._led_lock = threading.Lock()

        # Headlight material index
        self._headlight_mat_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_MATERIAL, f'{prefix}headlight')
        self._headlight_on = False
        self._headlight_lock = threading.Lock()

        # Light source indices (for LEDs that illuminate the scene)
        self._led_top_light_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_LIGHT, f'{prefix}led_top_light')
        self._led_bot_light_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_LIGHT, f'{prefix}led_bot_light')
        self._headlight_light_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_LIGHT, f'{prefix}headlight_light')

        # Flowdeck
        self._flowdeck = flowdeck
        self._flow_acc = 0.0
        self._tof_acc = 0.0
        self._last_flow_time = 0.0

        # Baro accumulator
        self._baro_acc = 0.0
        # Range accumulator
        self._range_acc = 0.0
        # Geom group mask for raycasting (include all groups)
        self._ray_geomgroup = np.ones(6, dtype=np.uint8)

        # UDP socket (firmware)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, fw_port))
        self._sock.settimeout(1.0)
        self._firmware_addr = None
        self._fw_port = fw_port

        # UDP socket (cflib passthrough)
        self._cflib_port = cflib_port
        self._cflib_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cflib_sock.bind((host, cflib_port))
        self._cflib_sock.settimeout(1.0)
        self._cflib_addr = None
        self._cflib_addr_lock = threading.Lock()
        self._firmware_to_cflib_q = queue.Queue(maxsize=20)
        self._cflib_to_firmware_q = queue.Queue(maxsize=20)

        self._running = False

        # Optional feature models
        self._noise_model = noise_model
        self._wind_model = wind_model
        self._ground_effect = ground_effect
        self._camera = camera_renderer

        print(f'[crazysim] Agent {agent_id}: fw_port={fw_port}  cflib_port={cflib_port}')

    def _pwm_to_rpm(self, pwm: int) -> float:
        """16-bit PWM → rotor RPM via crazyflow model.
        PWM maps linearly to thrust, then invert the rpm2thrust polynomial."""
        if pwm < 7000:
            return 0.0
        thrust = (pwm / 65535.0) * self._params.pwm_thrust_full
        a, b, c = self._params.rpm2thrust
        # Solve c*rpm² + b*rpm + (a - thrust) = 0
        disc = b * b - 4.0 * c * (a - thrust)
        if disc < 0:
            return 0.0
        rpm = (-b + math.sqrt(disc)) / (2.0 * c)
        return min(max(rpm, 0.0), self._params.max_rpm)

    def handshake(self):
        print(f'[crazysim] Agent {self.agent_id}: waiting for firmware on port {self._fw_port} ...')
        while True:
            try:
                data, addr = self._sock.recvfrom(64)
            except socket.timeout:
                continue
            if len(data) >= 1 and data[0] == 0xF3:
                self._firmware_addr = addr
                self._sock.sendto(bytes([0xF3]), addr)
                print(f'[crazysim] Agent {self.agent_id}: firmware connected from {addr}')
                return

    def start_threads(self):
        self._running = True
        threading.Thread(target=self._recv_thread, daemon=True).start()
        threading.Thread(target=self._recv_cflib_thread, daemon=True).start()
        threading.Thread(target=self._send_cflib_thread, daemon=True).start()

    def stop(self):
        self._running = False
        self._sock.close()
        self._cflib_sock.close()

    def _recv_thread(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 1:
                continue
            hdr = data[0]
            if hdr == CRTP_HDR_SIM and len(data) >= 9:
                # Motor PWM packet (SIM port, channel 0)
                m0, m1, m2, m3 = struct.unpack_from('<HHHH', data, 1)
                with self._motor_lock:
                    self._rpm_ref[:] = [self._pwm_to_rpm(p) for p in (m0, m1, m2, m3)]
            elif hdr == CRTP_HDR_LED and len(data) >= 5:
                # LED RGB packet (SIM port, channel 1): [pos, R, G, B]
                pos, r, g, b = data[1], data[2], data[3], data[4]
                rgb = [r / 255.0, g / 255.0, b / 255.0]
                with self._led_lock:
                    if pos == 0:    # bottom
                        self._led_bot_rgb[:] = rgb
                    elif pos == 1:  # top
                        self._led_top_rgb[:] = rgb
            elif hdr == CRTP_HDR_HEADLIGHT and len(data) >= 2:
                # Headlight on/off packet (SIM port, channel 2)
                with self._headlight_lock:
                    self._headlight_on = bool(data[1])
            else:
                with self._cflib_addr_lock:
                    has_cflib = self._cflib_addr is not None
                if has_cflib:
                    try:
                        self._firmware_to_cflib_q.put_nowait(data)
                    except queue.Full:
                        pass

    def _recv_cflib_thread(self):
        while self._running:
            try:
                data, addr = self._cflib_sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 1:
                continue
            with self._cflib_addr_lock:
                self._cflib_addr = addr
            if len(data) == 1 and data[0] == 0xFF:
                try:
                    self._cflib_sock.sendto(bytes([0xFF]), addr)
                except OSError:
                    pass
            if self._firmware_addr is not None:
                try:
                    self._cflib_to_firmware_q.put_nowait(data)
                except queue.Full:
                    pass

    def _send_cflib_thread(self):
        while self._running:
            try:
                data = self._firmware_to_cflib_q.get(timeout=0.1)
            except queue.Empty:
                continue
            with self._cflib_addr_lock:
                addr = self._cflib_addr
            if addr is not None:
                try:
                    self._cflib_sock.sendto(data, addr)
                except OSError:
                    pass

    def _compute_ground_effect(self) -> float:
        """Cheeseman-Bennett ground effect: thrust multiplier >= 1.0."""
        bid = self._body_id
        pos = self.data.xpos[bid]
        R = self.data.xmat[bid].reshape(3, 3)
        d_world = R @ np.array([0.0, 0.0, -1.0])
        geom_id = np.array([-1], dtype=np.int32)
        z = mujoco.mj_ray(
            self.model, self.data,
            pos, d_world,
            self._ray_geomgroup, 1, bid, geom_id,
        )
        pr = self._params.prop_radius
        if z < 0 or z > 4.0 * pr:
            return 1.0
        ratio = pr / (4.0 * max(z, 0.005))
        return 1.0 / (1.0 - ratio * ratio)

    def update_motors(self):
        p = self._params
        with self._motor_lock:
            rpm_ref = self._rpm_ref.copy()

        a_t, b_t, c_t = p.rpm2thrust
        a_q, b_q, c_q = p.rpm2torque

        ge_factor = self._compute_ground_effect() if self._ground_effect else 1.0

        rpm_prev = self._rpm.copy()
        for i in range(4):
            tau = p.tau_up if rpm_ref[i] >= self._rpm[i] else p.tau_down
            self._rpm[i] += (rpm_ref[i] - self._rpm[i]) * self.dt / tau
            self._rpm[i] = np.clip(self._rpm[i], 0.0, p.max_rpm)

            rpm = self._rpm[i]
            thrust = a_t + b_t * rpm + c_t * rpm * rpm
            thrust = max(thrust, 0.0) * ge_factor
            drag_torque = MOTOR_DIR[i] * (a_q + b_q * rpm + c_q * rpm * rpm)

            self.data.ctrl[self._act_force[i]] = thrust
            self.data.ctrl[self._act_torque[i]] = drag_torque

        # Store rotor acceleration (RPM/s) for gyroscopic z-torque
        self._rpm_dot = (self._rpm - rpm_prev) / self.dt

    def apply_aero_effects(self):
        """Apply aerodynamic drag via generalized forces (qfrc_applied).

        Uses qfrc_applied on the free joint DOFs instead of xfrc_applied
        to avoid interactions with MuJoCo's contact solver.

        Only active when motors are spinning (any RPM > 0) to avoid
        spurious forces from ground contact velocity noise.
        """
        # Skip when motors are off — ground contact velocity creates artifacts
        if np.all(self._rpm < 1.0):
            return

        p = self._params
        bid = self._body_id
        adr = self._qvel_adr

        # Body rotation matrix (3×3, body-to-world) and velocity from free joint
        R = self.data.xmat[bid].reshape(3, 3)
        lin_vel_world = self.data.qvel[adr:adr + 3]

        # Aerodynamic drag: use airspeed (body vel - wind) if wind model active
        if self._wind_model is not None:
            wind_world = self._wind_model.get_wind_velocity(
                self.data.xpos[bid], self.data.time)
            v_rel_world = lin_vel_world - wind_world
        else:
            v_rel_world = lin_vel_world
        v_body = R.T @ v_rel_world
        f_drag_body = p.drag_matrix @ v_body
        f_drag_world = R @ f_drag_body

        # Gyroscopic precession: τ = -ω_body × h_rotor
        # h_rotor = [0, 0, prop_inertia * Σ(dir_i * ω_i)]
        # Result is body-frame torque with zero z-component (pure precession)
        ang_vel_body = self.data.qvel[adr + 3:adr + 6]
        net_rotor_vel = float(np.sum(MOTOR_DIR * self._rpm)) * RPM_TO_RADS
        h_z = p.prop_inertia * net_rotor_vel
        gyro_torque = np.array([
            -ang_vel_body[1] * h_z,
             ang_vel_body[0] * h_z,
             0.0,
        ])

        # Apply as generalized forces on the free joint DOFs
        # Translational (world frame): drag force
        # Angular (body frame): gyroscopic precession torque
        self.data.qfrc_applied[adr:adr + 3] = f_drag_world
        self.data.qfrc_applied[adr + 3:adr + 6] = gyro_torque

    def read_imu(self):
        acc = self.data.sensordata[self._acc_adr: self._acc_adr + 3].copy()
        gyro = self.data.sensordata[self._gyro_adr: self._gyro_adr + 3].copy()
        return acc, gyro

    def read_pose(self):
        pos = self.data.xpos[self._body_id].copy()
        q_wxyz = self.data.xquat[self._body_id].copy()
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        return pos, q_xyzw

    def read_ranges(self) -> tuple[float, float, float, float, float]:
        """Cast 5 rays in body-frame directions, return distances [m] clamped to max range."""
        bid = self._body_id
        pos = self.data.xpos[bid]
        R = self.data.xmat[bid].reshape(3, 3)

        # Body-frame directions: front=+X, back=-X, left=+Y, right=-Y, up=+Z
        directions_body = (
            np.array([1.0, 0.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, -1.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
        )

        ranges = []
        for d_body in directions_body:
            d_world = R @ d_body
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(
                self.model, self.data,
                pos, d_world,
                self._ray_geomgroup,
                1,      # flg_static: include static geoms (floor, walls)
                bid,    # bodyexclude: skip drone's own body
                geom_id,
            )
            if dist < 0 or dist > RANGE_MAX_M:
                dist = RANGE_MAX_M
            ranges.append(dist)

        return tuple(ranges)

    def read_tof_down(self) -> float:
        """Cast a ray downward in body frame, return distance [m]."""
        bid = self._body_id
        pos = self.data.xpos[bid]
        R = self.data.xmat[bid].reshape(3, 3)
        d_world = R @ np.array([0.0, 0.0, -1.0])
        geom_id = np.array([-1], dtype=np.int32)
        dist = mujoco.mj_ray(
            self.model, self.data, pos, d_world,
            self._ray_geomgroup, 1, bid, geom_id,
        )
        if dist < 0 or dist > RANGE_MAX_M:
            dist = RANGE_MAX_M
        return dist

    def compute_flow(self) -> tuple[float, float, float]:
        """Compute optical flow dpixel_x, dpixel_y, dt from body state."""
        bid = self._body_id
        R = self.data.xmat[bid].reshape(3, 3)

        # Body velocity (world frame) → body frame
        vel_world = self.data.cvel[bid][3:6]  # linear velocity
        vel_body = R.T @ vel_world

        # Angular velocity in body frame
        omega_body = R.T @ self.data.cvel[bid][0:3]

        # Height above ground from TOF
        height = self.read_tof_down()
        if height < 0.1:
            height = 0.1

        # cos(tilt) = R[2,2] (body Z dot world Z)
        cos_tilt = R[2, 2]

        dt = 1.0 / FLOW_RATE_HZ
        scale = dt * FLOW_NPIX / FLOW_THETAPIX
        omega_factor = 1.25

        dpixelx = scale * (vel_body[0] / height * cos_tilt - omega_factor * omega_body[1])
        dpixely = scale * (vel_body[1] / height * cos_tilt + omega_factor * omega_body[0])

        # Simulated noise — lower than real PMW3901 (stdDev=2.0) because
        # MuJoCo ground plane has no surface-texture or lighting variance.
        # Firmware-side stdDev stays at 2.0 to match real hardware tuning.
        dpixelx += np.random.normal(0, 0.5)
        dpixely += np.random.normal(0, 0.5)

        return dpixelx, dpixely, dt

    def send_sensor_data(self):
        if not self._firmware_addr:
            return
        acc, gyro = self.read_imu()
        pos, quat = self.read_pose()

        # Apply sensor noise if enabled
        if self._noise_model is not None:
            acc, gyro = self._noise_model.apply_imu_noise(acc, gyro)

        self._sock.sendto(make_imu_packet(acc, gyro), self._firmware_addr)

        # Send pose only when NOT using flowdeck
        if not self._flowdeck:
            self._sock.sendto(make_pose_packet(pos, quat), self._firmware_addr)

        baro_period = 1.0 / BARO_RATE_HZ
        self._baro_acc += self.dt
        if self._baro_acc >= baro_period:
            self._baro_acc -= baro_period
            alt = pos[2]
            if self._noise_model is not None:
                alt = self._noise_model.apply_baro_noise(alt)
            self._sock.sendto(make_baro_packet(alt), self._firmware_addr)

        # Multi-ranger at RANGE_RATE_HZ
        range_period = 1.0 / RANGE_RATE_HZ
        self._range_acc += self.dt
        if self._range_acc >= range_period:
            self._range_acc -= range_period
            front, back, left, right, up = self.read_ranges()
            self._sock.sendto(make_range_packet(front, back, left, right, up),
                              self._firmware_addr)

        # Flowdeck: TOF + optical flow
        if self._flowdeck:
            tof_period = 1.0 / TOF_RATE_HZ
            self._tof_acc += self.dt
            if self._tof_acc >= tof_period:
                self._tof_acc -= tof_period
                dist = self.read_tof_down()
                self._sock.sendto(make_tof_packet(dist), self._firmware_addr)

            flow_period = 1.0 / FLOW_RATE_HZ
            self._flow_acc += self.dt
            if self._flow_acc >= flow_period:
                self._flow_acc -= flow_period
                dpx, dpy, fdt = self.compute_flow()
                self._sock.sendto(make_flow_packet(dpx, dpy, fdt),
                                  self._firmware_addr)

        # Camera rendering (must be called from main/physics thread)
        if self._camera is not None:
            self._camera.maybe_render(self.model, self.data, self.dt)

        # Drain cflib→firmware queue
        while not self._cflib_to_firmware_q.empty():
            try:
                pkt = self._cflib_to_firmware_q.get_nowait()
                self._sock.sendto(pkt, self._firmware_addr)
            except queue.Empty:
                break

    def update_led_materials(self):
        """Apply the latest LED RGB colors to MuJoCo materials and light sources."""
        with self._led_lock:
            bot_r, bot_g, bot_b = self._led_bot_rgb
            top_r, top_g, top_b = self._led_top_rgb

        # Bottom LED: material + light
        bot_on = (bot_r > 0 or bot_g > 0 or bot_b > 0)
        if self._led_bot_mat_id >= 0:
            self.model.mat_rgba[self._led_bot_mat_id] = [bot_r, bot_g, bot_b, 1.0 if bot_on else 0.0]
        if self._led_bot_light_id >= 0:
            self.model.light_active[self._led_bot_light_id] = bot_on
            self.model.light_diffuse[self._led_bot_light_id] = [bot_r, bot_g, bot_b]
            self.model.light_specular[self._led_bot_light_id] = [bot_r * 0.3, bot_g * 0.3, bot_b * 0.3]

        # Top LED: material + light
        top_on = (top_r > 0 or top_g > 0 or top_b > 0)
        if self._led_top_mat_id >= 0:
            self.model.mat_rgba[self._led_top_mat_id] = [top_r, top_g, top_b, 1.0 if top_on else 0.0]
        if self._led_top_light_id >= 0:
            self.model.light_active[self._led_top_light_id] = top_on
            self.model.light_diffuse[self._led_top_light_id] = [top_r, top_g, top_b]
            self.model.light_specular[self._led_top_light_id] = [top_r * 0.3, top_g * 0.3, top_b * 0.3]

        # Headlight: material + light
        with self._headlight_lock:
            hl_on = self._headlight_on
        if self._headlight_mat_id >= 0:
            self.model.mat_rgba[self._headlight_mat_id] = [1.0, 1.0, 0.95, 1.0 if hl_on else 0.0]
        if self._headlight_light_id >= 0:
            self.model.light_active[self._headlight_light_id] = hl_on
            if hl_on:
                self.model.light_diffuse[self._headlight_light_id] = [1.0, 1.0, 0.95]
                self.model.light_specular[self._headlight_light_id] = [0.3, 0.3, 0.3]
            else:
                self.model.light_diffuse[self._headlight_light_id] = [0, 0, 0]
                self.model.light_specular[self._headlight_light_id] = [0, 0, 0]


# ---------------------------------------------------------------------------
class CrazySimMuJoCo:
    """
    Multi-agent MuJoCo SITL simulator for Crazyflie firmware.

    One shared MuJoCo physics world with N drones. Each drone has its own
    UDP sockets (firmware + cflib) and firmware communication threads.
    Only one viewer window is used for visualization.
    """

    def __init__(self, model_path: str, host: str, base_port: int,
                 spawn_positions: list[tuple[float, float]],
                 visualize: bool = False, timestep: float = 0.001,
                 model_type: str = DEFAULT_MODEL_TYPE, *,
                 noise_model: SensorNoiseModel | None = None,
                 wind_model: WindModel | None = None,
                 ground_effect: bool = False,
                 flowdeck: bool = False,
                 downwash: bool = False,
                 camera_enabled: bool = False,
                 cam_width: int = 324, cam_height: int = 244,
                 cam_fps: float = 20.0,
                 cam_port: int | None = None):
        self.host = host
        self.visualize = visualize
        self.dt = timestep
        self.num_agents = len(spawn_positions)
        self._downwash = downwash
        self._flowdeck = flowdeck

        if model_type not in MOTOR_PARAMS:
            raise ValueError(f'Unknown model_type {model_type!r}. '
                             f'Choose from: {list(MOTOR_PARAMS)}')
        params = MOTOR_PARAMS[model_type]
        print(f'[crazysim] Model type  : {model_type}')
        print(f'[crazysim] Num agents  : {self.num_agents}')
        print(f'[crazysim] rpm2thrust  : {params.rpm2thrust}')
        print(f'[crazysim] rpm2torque  : {params.rpm2torque}')
        print(f'[crazysim] tau_up/down : {params.tau_up:.4f} / {params.tau_down:.4f} s')
        print(f'[crazysim] max_rpm     : {params.max_rpm:.0f}')

        print(f'[crazysim] mass       : {params.mass:.4f} kg')
        print(f'[crazysim] diaginertia: {params.diaginertia}')

        # Print enabled features
        features = []
        if noise_model is not None:
            features.append('sensor-noise')
        if wind_model is not None:
            features.append('wind')
        if ground_effect:
            features.append('ground-effect')
        if downwash:
            features.append('downwash')
        if camera_enabled:
            features.append('camera')
        if features:
            print(f'[crazysim] features   : {", ".join(features)}')

        # Build shared MuJoCo model with all drones
        self.model = _build_spec_multi_safe(model_path, spawn_positions, params)
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

        # Create per-drone agents
        self.agents: list[DroneAgent] = []
        for i in range(self.num_agents):
            fw_port = base_port + i
            cflib_port = fw_port + CFLIB_PORT_OFFSET
            # Per-agent noise and battery (independent state per drone)
            agent_noise = SensorNoiseModel(
                dt=self.dt,
            ) if noise_model is not None else None
            agent_camera = CameraRenderer(
                self.model, self.data, i,
                width=cam_width, height=cam_height, fps=cam_fps,
                cam_port=cam_port + i if cam_port is not None else None,
            ) if camera_enabled else None
            agent = DroneAgent(
                agent_id=i,
                model=self.model,
                data=self.data,
                params=params,
                host=host,
                fw_port=fw_port,
                cflib_port=cflib_port,
                dt=self.dt,
                noise_model=agent_noise,
                wind_model=wind_model,  # shared across agents (stateless reads)
                ground_effect=ground_effect,
                camera_renderer=agent_camera,
                flowdeck=flowdeck,
            )
            self.agents.append(agent)

        self._running = False

    def _apply_downwash(self):
        """Apply downwash force perturbations between drones."""
        for i, agent_below in enumerate(self.agents):
            for j, agent_above in enumerate(self.agents):
                if i == j:
                    continue
                dx = self.data.xpos[agent_below._body_id] - \
                    self.data.xpos[agent_above._body_id]
                horiz_dist = math.sqrt(dx[0]**2 + dx[1]**2)
                vert_dist = dx[2]  # positive means below is higher (wrong)
                if vert_dist > 0 or vert_dist < -2.0:
                    continue  # above is below or too far
                if horiz_dist > 0.3:
                    continue
                avg_rpm = float(np.mean(agent_above._rpm))
                if avg_rpm < 100:
                    continue
                intensity = (avg_rpm / agent_above._params.max_rpm) * \
                    math.exp(-horiz_dist / 0.1) / max(abs(vert_dist), 0.05)
                adr = agent_below._qvel_adr
                self.data.qfrc_applied[adr:adr + 3] += np.array([
                    np.random.normal(0, 0.002 * intensity),
                    np.random.normal(0, 0.002 * intensity),
                    -0.005 * intensity,
                ])

    def run(self):
        # Handshake all agents (in parallel threads to avoid blocking)
        handshake_threads = []
        for agent in self.agents:
            t = threading.Thread(target=agent.handshake)
            t.start()
            handshake_threads.append(t)
        for t in handshake_threads:
            t.join()

        self._running = True
        for agent in self.agents:
            agent.start_threads()

        def _step_and_send():
            """One physics step + CRTP packet dispatch for all agents."""
            for agent in self.agents:
                agent.update_motors()
                agent.apply_aero_effects()
                agent.update_led_materials()
            if self._downwash and self.num_agents > 1:
                self._apply_downwash()
            mujoco.mj_step(self.model, self.data)
            for agent in self.agents:
                agent.send_sensor_data()

        # Real-time factor tracking
        _rtf_interval = 1.0  # seconds between RTF updates
        _rtf_wall_start = time.perf_counter()
        _rtf_sim_start = self.data.time
        _rtf_value = 0.0

        def _update_rtf():
            nonlocal _rtf_wall_start, _rtf_sim_start, _rtf_value
            wall_now = time.perf_counter()
            wall_dt = wall_now - _rtf_wall_start
            if wall_dt >= _rtf_interval:
                sim_dt = self.data.time - _rtf_sim_start
                _rtf_value = sim_dt / wall_dt if wall_dt > 0 else 0.0
                _rtf_wall_start = wall_now
                _rtf_sim_start = self.data.time

        if self.visualize:
            _render_interval = 1.0 / 60.0  # 60 FPS render rate
            with mujoco.viewer.launch_passive(self.model, self.data,
                                              show_left_ui=False,
                                              show_right_ui=False) as v:
                # Force initial camera: look from behind drone toward +X.
                _cam_init_frames = 5
                _last_render = 0.0
                while v.is_running() and self._running:
                    t0 = time.perf_counter()
                    with v.lock():
                        _step_and_send()
                        if _cam_init_frames > 0:
                            v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                            v.cam.azimuth = 0.0
                            v.cam.elevation = -20.0
                            v.cam.distance = 3.0
                            v.cam.lookat[:] = [0.0, 0.0, 0.5]
                            _cam_init_frames -= 1
                    _update_rtf()
                    if t0 - _last_render >= _render_interval:
                        v.set_texts((
                            mujoco.mjtFontScale.mjFONTSCALE_150,
                            mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                            f'RTF: {_rtf_value:.2f}x',
                            f't={self.data.time:.1f}s',
                        ))
                        v.sync()
                        _last_render = t0
                    elapsed = time.perf_counter() - t0
                    sleep_t = self.dt - elapsed
                    if sleep_t > 0:
                        time.sleep(sleep_t)
            self._running = False
        else:
            try:
                while self._running:
                    t0 = time.perf_counter()
                    _step_and_send()
                    _update_rtf()
                    elapsed = time.perf_counter() - t0
                    sleep_t = self.dt - elapsed
                    if sleep_t > 0:
                        time.sleep(sleep_t)
            except KeyboardInterrupt:
                pass

        self._running = False
        for agent in self.agents:
            agent.stop()
        print('[crazysim] Done.')


# ---------------------------------------------------------------------------
_SPAWN_LIST_DIR = os.path.join(_HERE, '..', '..', 'drone_spawn_list')


def _parse_spawn_positions(agents_args: list[str] | None,
                           spawn_file: str | None) -> list[tuple[float, float]]:
    """Parse spawn positions from --agents args or --spawn-file.
    If spawn_file is not an absolute/relative path to an existing file,
    it is looked up in the shared drone_spawn_list directory."""
    if spawn_file is not None:
        if not os.path.isfile(spawn_file):
            shared = os.path.join(_SPAWN_LIST_DIR, spawn_file)
            if os.path.isfile(shared):
                spawn_file = shared
            else:
                raise FileNotFoundError(
                    f'Spawn file not found: {spawn_file}\n'
                    f'Also checked: {shared}')
        positions = []
        with open(spawn_file) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                fields = line.split(',')
                positions.append((float(fields[0]), float(fields[1])))
        return positions

    if agents_args is not None and len(agents_args) > 0:
        positions = []
        for arg in agents_args:
            parts = arg.split(',')
            positions.append((float(parts[0]), float(parts[1])))
        return positions

    # Default: single agent at origin
    return [(0.0, 0.0)]


def main():
    p = argparse.ArgumentParser(
        description='CrazySim MuJoCo SITL (multi-agent)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='\n'.join(
            [f'  {k:<12} → {v}' for k, v in MODEL_PATHS.items()]
        ))
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=19950,
                   help='Base firmware UDP port (agent N uses port+N)')
    p.add_argument('--model', default=None,
                   help='Path to MJCF model (default: auto-selected by --model-type)')
    p.add_argument('--model-type', default=None,
                   choices=list(MOTOR_PARAMS),
                   help=f'Drone variant (default: inferred from --model, else {DEFAULT_MODEL_TYPE})')
    p.add_argument('--vis', action='store_true', help='Launch passive viewer')
    p.add_argument('--dt', type=float, default=0.001, help='Physics timestep [s]')
    p.add_argument('agents', nargs='*', metavar='X,Y',
                   help='Spawn positions as X,Y pairs (e.g., 0,0 1,0 0,1)')
    p.add_argument('--spawn-file', default=None,
                   help='CSV file with spawn positions (one X,Y per line)')
    p.add_argument('--mass', type=float, default=None,
                   help='Override drone mass [kg]')
    p.add_argument('--scene', default=None,
                   help='Path to scene MJCF XML (default: scene.xml)')
    # --- Feature flags ---
    p.add_argument('--sensor-noise', action='store_true',
                   help='Enable sensor noise model (IMU + baro)')
    p.add_argument('--wind-speed', type=float, default=0.0,
                   help='Constant wind speed [m/s]')
    p.add_argument('--wind-direction', type=float, default=0.0,
                   help='Wind direction [deg], 0=+X, 90=+Y')
    p.add_argument('--gust-intensity', type=float, default=0.0,
                   help='Gust peak deviation [m/s]')
    p.add_argument('--turbulence', choices=['none', 'light', 'moderate', 'severe'],
                   default='none', help='Dryden turbulence level')
    p.add_argument('--flowdeck', action='store_true',
                   help='Simulate flowdeck (TOF + optical flow, disables pose)')
    p.add_argument('--ground-effect', action='store_true',
                   help='Enable Cheeseman-Bennett ground effect model')
    p.add_argument('--downwash', action='store_true',
                   help='Enable inter-drone downwash interaction')
    p.add_argument('--camera', action='store_true',
                   help='Enable AI-deck camera (CPX WiFi streaming on TCP)')
    p.add_argument('--cam-width', type=int, default=324,
                   help='Camera image width [px] (default: 324, matches AI-deck)')
    p.add_argument('--cam-height', type=int, default=244,
                   help='Camera image height [px] (default: 244, matches AI-deck)')
    p.add_argument('--cam-fps', type=float, default=20.0,
                   help='Camera frame rate [Hz] (default: 20)')
    p.add_argument('--cam-port', type=int, default=None,
                   help='Base TCP port for internal frame server '
                        '(default: 5100, agent N uses port+N). '
                        'Use crazysim_cpx.py to bridge to CPX clients.')
    args = p.parse_args()

    # Override scene XML if provided
    global _SCENE_XML
    if args.scene is not None:
        scene_path = args.scene
        if not os.path.isabs(scene_path) and not os.path.isfile(scene_path):
            alt = os.path.join(_HERE, scene_path)
            if os.path.isfile(alt):
                scene_path = alt
        _SCENE_XML = scene_path

    # Resolve model type
    model_type = args.model_type
    if model_type is None and args.model is not None:
        model_type = _infer_model_type(args.model)
    if model_type is None:
        model_type = DEFAULT_MODEL_TYPE

    # Apply mass override if provided
    if args.mass is not None:
        from dataclasses import replace
        MOTOR_PARAMS[model_type] = replace(MOTOR_PARAMS[model_type], mass=args.mass)

    # Resolve model path (default paths are relative to this script)
    model_path = args.model
    if model_path is None:
        model_path = os.path.join(_HERE, MODEL_PATHS[model_type])
    elif not os.path.isabs(model_path):
        # If user-provided relative path doesn't exist, try relative to _HERE
        if not os.path.isfile(model_path):
            alt = os.path.join(_HERE, model_path)
            if os.path.isfile(alt):
                model_path = alt

    # Parse spawn positions
    spawn_positions = _parse_spawn_positions(args.agents, args.spawn_file)

    # Build optional feature models
    noise_model = SensorNoiseModel(
        dt=args.dt,
    ) if args.sensor_noise else None

    wind_active = (args.wind_speed > 0 or args.gust_intensity > 0
                   or args.turbulence != 'none')
    wind_model = WindModel(
        speed=args.wind_speed, direction_deg=args.wind_direction,
        gust_intensity=args.gust_intensity, turbulence=args.turbulence,
        dt=args.dt,
    ) if wind_active else None

    CrazySimMuJoCo(
        model_path=model_path,
        host=args.host,
        base_port=args.port,
        spawn_positions=spawn_positions,
        visualize=args.vis,
        timestep=args.dt,
        model_type=model_type,
        noise_model=noise_model,
        wind_model=wind_model,
        ground_effect=args.ground_effect,
        flowdeck=args.flowdeck,
        downwash=args.downwash,
        camera_enabled=args.camera,
        cam_width=args.cam_width,
        cam_height=args.cam_height,
        cam_fps=args.cam_fps,
        cam_port=args.cam_port,
    ).run()


if __name__ == '__main__':
    main()
