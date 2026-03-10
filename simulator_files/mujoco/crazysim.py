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
from dataclasses import dataclass

import mujoco
import mujoco.viewer
import numpy as np

# ---------------------------------------------------------------------------
# CRTP constants (CrtpUtils.h)
# ---------------------------------------------------------------------------
CRTP_PORT_SIM    = 0x09
CRTP_PORT_LOC    = 0x06
CRTP_HDR_SIM     = (CRTP_PORT_SIM << 4) | 0           # 0x90
CRTP_HDR_LOC     = (CRTP_PORT_LOC << 4) | (1 << 2) | 1  # 0x65

SENSOR_GYRO_ACC  = 0
SENSOR_BARO      = 2
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
        result[name] = MotorParams(
            rpm2thrust=rpm2thrust,
            rpm2torque=tuple(p['rpm2torque']),
            tau_up=tau,
            tau_down=tau,
            pwm_thrust_full=p['thrust_max'],
            max_rpm=_max_rpm(rpm2thrust, p['thrust_max']),
            mass=p['mass'],
            diaginertia=(J[0][0], J[1][1], J[2][2]),
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


# ---------------------------------------------------------------------------
# Directory containing this script — used to resolve scene.xml
_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_XML = os.path.join(_HERE, 'scene.xml')


def _patch_drone_spec(spec: mujoco.MjSpec, params: MotorParams) -> None:
    """Patch an upstream drone-models spec with CrazySim requirements.

    - Override mass and inertia from params.toml (authoritative source)
    - Add IMU site and accelerometer/gyro sensors
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
                 cflib_port: int, dt: float):
        self.agent_id = agent_id
        self.model = model
        self.data = data
        self.dt = dt
        self._params = params
        prefix = f'cf{agent_id}_'

        # Body ID
        self._body_id = model.body(f'{prefix}drone').id

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
        self._motor_lock = threading.Lock()

        # Baro accumulator
        self._baro_acc = 0.0

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
            port_nibble = (data[0] >> 4) & 0x0F
            if port_nibble == CRTP_PORT_SIM and len(data) >= 9:
                m0, m1, m2, m3 = struct.unpack_from('<HHHH', data, 1)
                with self._motor_lock:
                    self._rpm_ref[:] = [self._pwm_to_rpm(p) for p in (m0, m1, m2, m3)]
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

    def update_motors(self):
        p = self._params
        with self._motor_lock:
            rpm_ref = self._rpm_ref.copy()

        a_t, b_t, c_t = p.rpm2thrust
        a_q, b_q, c_q = p.rpm2torque

        for i in range(4):
            tau = p.tau_up if rpm_ref[i] >= self._rpm[i] else p.tau_down
            self._rpm[i] += (rpm_ref[i] - self._rpm[i]) * self.dt / tau
            self._rpm[i] = np.clip(self._rpm[i], 0.0, p.max_rpm)

            rpm = self._rpm[i]
            thrust = a_t + b_t * rpm + c_t * rpm * rpm
            thrust = max(thrust, 0.0)
            drag_torque = MOTOR_DIR[i] * (a_q + b_q * rpm + c_q * rpm * rpm)

            self.data.ctrl[self._act_force[i]] = thrust
            self.data.ctrl[self._act_torque[i]] = drag_torque

    def read_imu(self):
        acc = self.data.sensordata[self._acc_adr: self._acc_adr + 3].copy()
        gyro = self.data.sensordata[self._gyro_adr: self._gyro_adr + 3].copy()
        return acc, gyro

    def read_pose(self):
        pos = self.data.xpos[self._body_id].copy()
        q_wxyz = self.data.xquat[self._body_id].copy()
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        return pos, q_xyzw

    def send_sensor_data(self):
        if not self._firmware_addr:
            return
        acc, gyro = self.read_imu()
        pos, quat = self.read_pose()

        self._sock.sendto(make_imu_packet(acc, gyro), self._firmware_addr)
        self._sock.sendto(make_pose_packet(pos, quat), self._firmware_addr)

        baro_period = 1.0 / BARO_RATE_HZ
        self._baro_acc += self.dt
        if self._baro_acc >= baro_period:
            self._baro_acc -= baro_period
            self._sock.sendto(make_baro_packet(pos[2]), self._firmware_addr)

        # Drain cflib→firmware queue
        while not self._cflib_to_firmware_q.empty():
            try:
                pkt = self._cflib_to_firmware_q.get_nowait()
                self._sock.sendto(pkt, self._firmware_addr)
            except queue.Empty:
                break


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
                 model_type: str = DEFAULT_MODEL_TYPE):
        self.host = host
        self.visualize = visualize
        self.dt = timestep
        self.num_agents = len(spawn_positions)

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

        # Build shared MuJoCo model with all drones
        self.model = _build_spec_multi_safe(model_path, spawn_positions, params)
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

        # Create per-drone agents
        self.agents: list[DroneAgent] = []
        for i in range(self.num_agents):
            fw_port = base_port + i
            cflib_port = fw_port + CFLIB_PORT_OFFSET
            agent = DroneAgent(
                agent_id=i,
                model=self.model,
                data=self.data,
                params=params,
                host=host,
                fw_port=fw_port,
                cflib_port=cflib_port,
                dt=self.dt,
            )
            self.agents.append(agent)

        self._running = False

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
    args = p.parse_args()

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

    CrazySimMuJoCo(
        model_path=model_path,
        host=args.host,
        base_port=args.port,
        spawn_positions=spawn_positions,
        visualize=args.vis,
        timestep=args.dt,
        model_type=model_type,
    ).run()


if __name__ == '__main__':
    main()
