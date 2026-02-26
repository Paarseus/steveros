# Motor Oscillation Postmortem: RS03 Stick-Slip Fix

## Problem

Motor 21 (RS03) exhibited stick-slip oscillation during large movements (±45°).
The motor would stick due to gearbox friction, build corrective torque via the
firmware PD loop, then lurch 10-25° past the target when static friction broke,
snap back, and repeat. This produced audible clicking and vibration that worsened
further from the starting position.

Small movements (±5°) worked fine. The problem only appeared at larger amplitudes
where the motor needed sustained torque to overcome gearbox friction throughout
the trajectory.

## Root Cause

Two issues combined to cause the oscillation:

### 1. Excessive Kp with no velocity feedforward

The initial configuration used `Kp=500, Kd=5` with `v_des=0`. With the RS03's
gearbox friction (~5-8 Nm static), a pure position-error PD controller builds
large corrective torques while the motor is stuck. When static friction breaks,
the accumulated position error drives an overshoot, which then reverses — creating
a limit cycle.

Lowering to `Kp=200` reduced the torque buildup during stiction periods, keeping
the system stable.

### 2. Step position commands instead of smooth trajectories

The hardware interface was sending raw position commands from the controller with
`v_des=0`, relying entirely on the motor firmware's PD loop to track. For large
movements this means the full position error (e.g. 45° = 0.785 rad) appears
instantly, producing maximum torque from the first timestep.

Smooth trajectory generation (cosine blend or spline interpolation) with velocity
feedforward gradually increases `p_des` and provides `v_des` so the motor tracks
a moving target rather than fighting to reach a distant one.

## What was NOT the cause

**Firmware limit parameters** (`limit_torque`, `limit_cur`, `limit_spd`) were
initially suspected of capping motor output at ~12 Nm (CyberGear default). A
raw CAN parameter read on a fresh power cycle showed the actual firmware defaults
are already at rated maximums:

| Parameter | Default | Rated Max |
|-----------|---------|-----------|
| limit_torque (0x700B) | 60.0 Nm | 60.0 Nm |
| limit_cur (0x7018) | 43.0 A | 27.0 A (spec) / 43.0 A (K-Scale) |
| limit_spd (0x7017) | 10.0 rad/s | 20.0 rad/s |

An earlier diagnostic that seemed to show firmware limits were the fix was
actually confounded by simultaneously changing the gains and trajectory shape.
The parameter reads returning zeros were due to a socket race condition (the
driver's receive thread consumed the response frames before the diagnostic's
raw socket could read them).

## Fix

### 1. Velocity command passthrough (steveros_hardware.cpp)

Added a velocity command interface so `joint_trajectory_controller` can send
`v_des` alongside `p_des`. The hardware interface passes both through to the
MIT command frame:

- Export `HW_IF_VELOCITY` command interface
- Accept position + velocity command interfaces (position required, velocity optional)
- Pass `command_velocity` as `v_des` in the MIT command

### 2. Controller configuration (test_single_motor_controllers.yaml)

Configured `joint_trajectory_controller` with both position and velocity
command interfaces:

```yaml
command_interfaces:
  - position
  - velocity
```

The controller uses spline interpolation to generate smooth position and velocity
profiles between trajectory waypoints.

### 3. Gain tuning (test_single_motor.urdf.xacro)

Set RS03 gains to `Kp=200, Kd=5` based on testing. This provides sufficient
stiffness for position holding while avoiding stick-slip oscillation.

For reference, EDULITE_A3 (Robstride's reference implementation) uses `Kp=60,
Kd=3.5` for RS01/RS02 motors (which have less gearbox friction than RS03).

## Verification

Full ±45° movement test with `Kp=200, Kd=5`, cosine trajectory, and velocity
feedforward achieved **0.45° max tracking error** with no stick-slip, no audible
noise, and smooth motion throughout.

The same test through `joint_trajectory_controller` with rqt slider control
confirmed smooth operation via the standard ros2_control stack.

## Architecture Note

Trajectory smoothing belongs in the controller layer (e.g. `joint_trajectory_controller`),
not in the hardware interface. The hardware interface is a passthrough: it receives
`p_des` and `v_des` from the controller and sends them directly to the motor's
MIT mode frame. This matches the standard ros2_control architecture.

EDULITE_A3's approach of putting S-curve trajectory generation inside the hardware
interface is non-standard and was not adopted.

## Recommended Gains

| Motor Type | Kp | Kd | Notes |
|-----------|-----|-----|-------|
| RS01 | 60 | 3.5 | Per EDULITE_A3 reference |
| RS02 | 60 | 3.5 | Per EDULITE_A3 reference |
| RS03 | 200 | 5.0 | Verified on motor 21 |
| RS04 | TBD | TBD | Needs testing (higher gearbox friction) |
