# Rover Models for CrazySim

Rover models that can be attached to the CrazySim MuJoCo scene.

## Usage

```bash
# From the mujoco directory:
python3 crazysim.py --vis \
    --rover-model rosmaster_x3.xml \
    --rover-pos 1.5,0 \
    --rover-heading 0 \
    0,0
```

This spawns one drone at (0,0) and the X3 rover at (1.5, 0).

## Files

- `rosmaster_x3.xml` — MuJoCo model of the Yahboom RosMaster X3 with landing pad collision
- `mecanum_dynamics.py` — NumPy mecanum drive dynamics (kinematic, not MuJoCo physics)

## How it works

The rover body is attached to the MuJoCo scene as a freejoint body. Its position is
driven kinematically by `mecanum_dynamics.py` (RK4 integration), not by MuJoCo forces.
However, the collision geoms on the landing pad are active, so the drone physically
interacts with the rover surface when landing.

The `RoverSim` class in `crazysim.py` handles:
- Stepping mecanum dynamics each physics step
- Writing rover qpos/qvel to MuJoCo before `mj_step()`
- Running a ROS2 node that publishes `/rover/odom` and `/vel_raw`, subscribes to `/cmd_vel`
