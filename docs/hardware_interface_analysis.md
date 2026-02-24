# SteveROS Hardware Interface Analysis & Rewrite Plan

**Date**: 2026-02-24
**Status**: Pre-implementation analysis complete

---

## 1. Problem Statement

During single motor testing of motor 21 (right shoulder pitch, Robstride RS-series) via `ros2_control`, the motor produced a loud noise on startup and slammed toward position zero without any user command. After ~47 seconds, a feedback timeout triggered and the controller manager deactivated the hardware.

### Root Cause: Snap-to-Zero Activation Bug

The current `on_activate()` implementation in `steveros_hardware.cpp`:

1. Sends motor enable (Type 3 CAN frame)
2. Waits only **10ms** for feedback
3. Attempts a non-blocking drain of the CAN receive buffer
4. If no feedback arrives (common with only 10ms window), `hw_positions_` remains at **0.0** (its initialized value)
5. Sets `hw_commands_[i] = hw_positions_[i]` — commanding position **0.0**
6. Begins Kp ramp from 0 to 40 over 1 second, driving the motor to position 0.0
7. Motor physically at a different position slams toward zero

The motor noise was the motor fighting to reach position 0.0 from its actual position under increasing stiffness (Kp ramp).

### Log Evidence

From `/home/nemo/.ros/log/2026-02-24-14-31-51-698745-nemo-227114/launch.log`:

```
[INFO]  Activating: enabling 1 motors...
[INFO]  Activated: all motors enabled (Kp ramp over 1.0s).
        ... 47 seconds of commanding position 0.0 ...
[ERROR] Feedback timeout on motor 21 ('dof_right_shoulder_pitch_03'): 0.500s since last feedback
[ERROR] Deactivating following hardware components as their read cycle resulted in an error
```

---

## 2. Reference Analysis

We analyzed two reference implementations to determine the best approach for fixing our hardware interface.

### 2.1 EDULITE_A3 (RobStride Official 6-DOF Arm)

**Repository**: https://github.com/RobStride/EDULITE_A3
**Relevance**: Uses the **exact same** Robstride MIT mode CAN protocol as SteveROS

#### Architecture

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| CAN Driver | `robstride_can_driver.cpp` | ~466 | Low-level CAN I/O with background receive thread |
| Hardware Interface | `rs_a3_hardware.cpp` | ~1386 | ros2_control SystemInterface plugin |
| S-Curve Generator | `s_curve_generator.cpp` | ~300 | Trajectory smoothing |
| **Total** | | **~2152** | |

#### Key Patterns (Protocol Layer)

**3-Step Motor Activation Sequence** — The critical pattern missing from SteveROS:

```
Step 0: Clear all motor faults
        disableMotor(motor_id, clear_fault=true)
        → Sends Type 4 frame with data[0]=1
        → 50ms per motor + 200ms settle time

Step 1: Set run mode + soft enable
        setRunMode(motor_id, MOTION_CONTROL)
        → Type 18 frame, param index 0x7005, value 0
        enableMotor(motor_id)
        → Type 3 frame
        sendMotionControl(pos=0, vel=0, Kp=0, Kd=4, torque=0)
        → Soft enable: motor is enabled but exerts NO position force
        → 30-50ms delays between operations

Step 2: Read initial positions from feedback cache
        → Background thread has been collecting feedback since Step 0

Step 3: Switch to position hold
        sendMotionControl(pos=current, vel=0, Kp=normal, Kd=normal, torque=0)
        → Now commanding the ACTUAL position, not 0.0
```

**CAN Send Retries** — Essential for 20-motor bus congestion:

```cpp
for (int retry = 0; retry < 10; retry++) {
    ssize_t nbytes = ::write(socket_, &frame, sizeof(frame));
    if (nbytes == sizeof(frame)) return true;
    if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(retry_delay_us * (retry + 1));  // Progressive backoff
        continue;
    }
    break;  // Non-retryable error
}
```

**Background Receive Thread** — Uses `poll()` with 10ms timeout:

```cpp
void receiveThread() {
    while (running_) {
        struct pollfd pfd = {socket_, POLLIN, 0};
        if (poll(&pfd, 1, 10) > 0) {
            struct can_frame frame;
            if (recv(socket_, &frame, sizeof(frame), 0) == sizeof(frame)) {
                auto fb = decode(frame);
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                feedback_cache_[fb.motor_id] = fb;
            }
        }
    }
}
```

**Per-Model Motor Parameters**:

```cpp
MotorParams getMotorParams(MotorType type) {
    switch (type) {
        case RS00: return {vel:±30, torque:±0.3, kp:0-500, kd:0-5};
        case EL05: return {vel:±30, torque:±1.4, kp:0-500, kd:0-5};
        // SteveROS needs to add: RS01, RS02, RS03, RS04
    }
}
```

**Fault Clearing Protocol**:

The Type 4 (stop) frame with `data[0]=1` clears motor fault flags. This must be sent before enabling a motor that was previously in a fault state. Without this, enable commands are silently ignored by the motor controller.

#### Strengths

- Exact same CAN protocol — every encode/decode function is directly applicable
- Battle-tested 3-step activation sequence solves the snap-to-zero bug
- Send retries handle CAN bus congestion with multiple motors
- Thread-safe feedback caching with mutex protection
- Per-model parameter lookup prevents encoding errors

#### Weaknesses

- 1386-line monolith hardware interface mixes concerns:
  - Pinocchio RNEA gravity compensation
  - S-curve trajectory generation
  - 2-stage IIR velocity feedforward filter
  - 5 debug topic publishers with separate spin thread
  - Zero-torque teach mode service
  - Manual YAML parsing for inertia calibration
- These features belong in controller plugins, not the hardware interface
- Code quality is functional but not cleanly separated

#### Assessment

Good engineering in the protocol and safety layers; poor separation of concerns in the hardware interface. The CAN driver layer is production-quality. The hardware interface is a cautionary tale of feature creep — putting application logic where I/O logic should be.

---

### 2.2 ODrive (ODrive Motor Controller ROS2 Interface)

**Repository**: https://github.com/odrive-robotics/ros_odrive
**Relevance**: Different motor protocol (ODrive CAN Simple), but excellent ros2_control architecture

#### Architecture

| Component | File | Lines | Purpose |
|-----------|------|-------|---------|
| Hardware Interface | `odrive_hardware_interface.cpp` | ~250 | ros2_control SystemInterface (class + impl in one file) |
| Socket CAN | `socket_can.cpp` | ~137 | SocketCAN abstraction with SOCK_NONBLOCK |
| Epoll Loop | `epoll_event_loop.cpp` | ~100 | Event-driven CAN receive |
| **Total** | | **~487** | |

#### Key Patterns (Architecture)

**NAN-Initialized States** — The single most important safety pattern:

```cpp
struct Axis {
    double pos_estimate_ = std::numeric_limits<double>::quiet_NaN();
    double vel_estimate_ = std::numeric_limits<double>::quiet_NaN();
    double torque_estimate_ = std::numeric_limits<double>::quiet_NaN();
    // ...
};
```

When states are NAN, ros2_control and downstream controllers know they have no valid data yet. The current SteveROS initializes everything to 0.0, which is the root cause of the snap-to-zero bug — the controller sees position=0.0 and commands the motor to hold there.

**Deferred Motor Enable** — Motors enabled in `perform_command_mode_switch()`, not `on_activate()`:

```cpp
// From odrive_hardware_interface.cpp:
// "This can be called several seconds before the controller finishes starting.
//  Therefore we enable the ODrives only in perform_command_mode_switch()."
hardware_interface::return_type perform_command_mode_switch(...) {
    // Enable motor here — controller is definitely ready
}
```

This ensures the controller is fully initialized before the motor receives its first command.

**Per-Joint Struct** — Clean data encapsulation replacing parallel vectors:

```cpp
struct Axis {
    // Config
    uint64_t node_id;
    // States (read from motor)
    double pos_estimate_ = NAN;
    double vel_estimate_ = NAN;
    double torque_estimate_ = NAN;
    // Commands (sent to motor)
    double pos_setpoint_ = 0.0;
    double vel_setpoint_ = 0.0;
    double torque_setpoint_ = 0.0;
    // Interface
    bool is_connected_ = false;
    void send(SocketCanIntf& intf);
};
```

**Minimal read()/write()** — I/O only, no application logic:

```cpp
return_type read(...) {
    // Drain CAN frames, update state
}
return_type write(...) {
    // Encode commands, send CAN frames
}
```

No filtering, no gravity compensation, no debug publishing — those belong in controller plugins.

#### Strengths

- 250 lines for a complete hardware interface — no bloat
- NAN initialization prevents stale-data bugs
- Deferred enable prevents timing races
- Clean separation of concerns (I/O only in hardware interface)
- Per-joint struct improves cache locality and code clarity

#### Weaknesses

- Completely different CAN protocol (ODrive CAN Simple, not Robstride MIT mode) — zero code reusability for protocol layer
- No send retries — single `write()` call, only stderr on failure
- No feedback timeout checking
- No fault clearing sequence
- No per-model parameter support

#### Assessment

Excellent architectural template. The cleanest ros2_control hardware interface we found. But protocol-irrelevant — we can't reuse any of the CAN communication code.

---

## 3. Comparative Analysis

### Scoring Matrix

| Criterion | EDULITE_A3 | ODrive | Current SteveROS |
|-----------|:----------:|:------:|:----------------:|
| Protocol match | **10/10** | 1/10 | 10/10 |
| Architecture cleanliness | 4/10 | **9/10** | 6/10 |
| Motor activation safety | **9/10** | 7/10 | 3/10 |
| Scalability (20 motors) | 6/10 | **7/10** | 5/10 |
| Robustness (retries, threading) | **8/10** | 5/10 | 6/10 |
| Minimal complexity | 3/10 | **9/10** | 7/10 |
| Adoptability | **7/10** (protocol) | **9/10** (architecture) | — |

### Recommendation: Hybrid Approach

Neither repository alone is the best reference. The optimal rewrite combines:

**From EDULITE_A3 — Protocol Layer:**
- `RobstrideCanDriver` class with background receive thread, send retries, mutex safety
- 3-step activation sequence (clear faults → soft enable → position hold)
- Per-model `MotorParams` lookup (extended for RS01-RS04)
- Fault clearing protocol (`data[0]=1` on Type 4)
- Type 18 parameter write (for `setRunMode()`)
- 50us inter-frame delays for CAN bus congestion management

**From ODrive — Architecture:**
- Per-joint `JointState` struct with NAN-initialized states
- Minimal `read()`/`write()` — I/O only, no application logic
- Deferred motor enable concept
- Clean single-purpose hardware interface

**Excluded from both:**
- EDULITE's Pinocchio gravity compensation → controller plugin
- EDULITE's S-curve generator → use JointTrajectoryController's built-in interpolation
- EDULITE's velocity IIR filter → controller responsibility
- EDULITE's debug topic publishers → use ros2_control built-in state publishing
- ODrive's epoll event loop → `poll()` in receive thread is sufficient
- EDULITE's `open_loop_control: true` → run closed-loop with proper filtering

---

## 4. Current SteveROS Hardware Interface

### File Inventory

| File | Lines | Status |
|------|-------|--------|
| `steveros_hardware/src/steveros_hardware.cpp` | 483 | Needs rewrite |
| `steveros_hardware/include/steveros_hardware/steveros_hardware.hpp` | 131 | Needs rewrite |
| `steveros_hardware/include/steveros_hardware/robstride_protocol.hpp` | 188 | Keep + extend |
| `steveros_hardware/include/steveros_hardware/visibility_control.h` | — | Keep as-is |
| `steveros_hardware/CMakeLists.txt` | 53 | Minor edit |
| `steveros_hardware/steveros_hardware.xml` | 7 | Keep as-is |

### What Works Today

- CAN protocol encoding/decoding in `robstride_protocol.hpp` is **correct** (validated against EDULITE_A3)
- URDF parameter parsing (motor_id, sign, zero_offset, kp, kd, max_torque)
- CAN receive filter for Type 2 feedback frames
- Feedback timeout detection in `read()`
- Kp ramp on activation
- Motor-to-joint / joint-to-motor coordinate transforms
- Clean deactivation (zero-torque → stop)

### What's Broken or Missing

| Issue | Severity | Fix |
|-------|----------|-----|
| States initialized to 0.0 (not NAN) | **Critical** | NAN initialization |
| No fault clearing before enable | **Critical** | Type 4 with `data[0]=1` |
| No run mode setting | **High** | Type 18, param 0x7005 |
| 10ms activation window too short | **High** | Background thread + 3-step sequence |
| No soft enable (Kp=0) phase | **High** | Send Kp=0 command before position hold |
| No CAN send retries | **Medium** | Retry loop with ENOBUFS backoff |
| Blocking socket (no background receive) | **Medium** | Background thread with `poll()` |
| Hardcoded RS01/RS02 protocol ranges | **Medium** | Per-model `MotorParams` lookup |
| Parallel vectors (cache fragmentation) | **Low** | Per-joint struct |
| `write()` ignores send failures | **Low** | Check return value |

---

## 5. Rewrite Architecture

### Target Structure

```
steveros_hardware/
  include/steveros_hardware/
    robstride_can_driver.hpp     NEW   (~130 lines)
    steveros_hardware.hpp        REWRITE (~80 lines)
    robstride_protocol.hpp       KEEP + extend (~20 new lines)
    visibility_control.h         KEEP
  src/
    robstride_can_driver.cpp     NEW   (~350 lines)
    steveros_hardware.cpp        REWRITE (~280 lines)
  steveros_hardware.xml          KEEP
  CMakeLists.txt                 EDIT  (~3 lines changed)
```

**Estimated total**: ~860 lines (comparable to current 858, but properly structured)

### Layer Responsibilities

```
┌─────────────────────────────────────────────┐
│  ros2_control Controller Manager (100 Hz)   │
│  ┌─────────────────────────────────────────┐│
│  │  JointTrajectoryController              ││
│  │  (interpolation, trajectory execution)  ││
│  └────────────────┬────────────────────────┘│
│                   │ position commands        │
│  ┌────────────────▼────────────────────────┐│
│  │  SteveROSHardware (SystemInterface)     ││
│  │  - Lifecycle management                 ││
│  │  - State export (NAN-initialized)       ││
│  │  - Coordinate transforms (sign/offset)  ││
│  │  - Kp ramp on activation               ││
│  │  - Feedback timeout checking            ││
│  └────────────────┬────────────────────────┘│
│                   │ motor commands           │
│  ┌────────────────▼────────────────────────┐│
│  │  RobstrideCanDriver                     ││
│  │  - SocketCAN open/close                 ││
│  │  - Background receive thread (poll)     ││
│  │  - Send with retries (ENOBUFS backoff)  ││
│  │  - Thread-safe feedback cache           ││
│  │  - Per-model MotorParams lookup         ││
│  │  - Enable/disable/fault-clear protocol  ││
│  └────────────────┬────────────────────────┘│
│                   │ CAN frames               │
│  ┌────────────────▼────────────────────────┐│
│  │  robstride_protocol.hpp (header-only)   ││
│  │  - MIT command encoding (Type 1)        ││
│  │  - Feedback decoding (Type 2)           ││
│  │  - Enable/Stop encoding (Type 3/4)      ││
│  │  - Param write encoding (Type 18)       ││
│  │  - float↔uint16 conversion helpers      ││
│  └────────────────┬────────────────────────┘│
│                   │                          │
│              SocketCAN (can0)                │
└─────────────────────────────────────────────┘
```

### Activation Sequence (Fixed)

```
on_activate():
  ┌──────────────────────────────────────────┐
  │ 1. CLEAR FAULTS                          │
  │    for each motor:                       │
  │      disableMotor(id, clear_fault=true)  │
  │      sleep 50ms                          │
  │    sleep 200ms settle                    │
  ├──────────────────────────────────────────┤
  │ 2. SOFT ENABLE (Kp=0)                   │
  │    for each motor:                       │
  │      setRunMode(id, MOTION_CONTROL)      │
  │      enableMotor(id)                     │
  │      sendMotionControl(Kp=0, Kd=kd)     │
  │      sleep 50ms                          │
  ├──────────────────────────────────────────┤
  │ 3. READ INITIAL POSITIONS               │
  │    for each motor:                       │
  │      fb = getFeedback(id)               │
  │      if fb.is_valid:                     │
  │        joint.position = transform(fb)    │
  │        joint.cmd_position = position     │
  │      else: return ERROR                  │
  ├──────────────────────────────────────────┤
  │ 4. START Kp RAMP                         │
  │    activation_time = now                 │
  │    ramping = true                        │
  │    // Kp ramps 0→target over 1 second    │
  │    // but commanding ACTUAL position     │
  └──────────────────────────────────────────┘
```

---

## 6. Per-Model Motor Parameters

The Robstride MIT mode protocol encodes values as uint16 within model-specific ranges. Using wrong ranges causes incorrect position/velocity/torque commands.

| Parameter | RS01 | RS02 | RS03 | RS04 | EL05 |
|-----------|------|------|------|------|------|
| Position (rad) | [-4π, 4π] | [-4π, 4π] | [-4π, 4π] | [-4π, 4π] | [-4π, 4π] |
| Velocity (rad/s) | [-44, 44] | [-44, 44] | [-20, 20] | [-15, 15] | [-30, 30] |
| Torque (Nm) | [-12, 12] | [-12, 12] | [-60, 60] | [-120, 120] | [-1.4, 1.4] |
| Kp | [0, 500] | [0, 500] | [0, 500] | [0, 500] | [0, 500] |
| Kd | [0, 5] | [0, 5] | [0, 5] | [0, 5] | [0, 5] |

*Note: RS03/RS04 ranges need verification against official Robstride documentation.*

---

## 7. CAN Protocol Frame Types

### Type 1 — MIT Mode Command (host → motor)

```
CAN ID (29-bit extended):
  [28:24] = 0x01 (Type 1)
  [23:8]  = torque feedforward (uint16, encoded)
  [7:0]   = target motor_id

Data (8 bytes):
  [0:1] = position (uint16, big-endian)
  [2:3] = velocity (uint16, big-endian)
  [4:5] = Kp (uint16, big-endian)
  [6:7] = Kd (uint16, big-endian)
```

### Type 2 — Feedback (motor → host)

```
CAN ID (29-bit extended):
  [28:24] = 0x02 (Type 2)
  [23:22] = motor mode (2 bits: 0=Reset, 1=Cali, 2=Run)
  [21:16] = fault flags (6 bits)
  [15:8]  = motor_id
  [7:0]   = host_id echo

Data (8 bytes):
  [0:1] = position (uint16, big-endian)
  [2:3] = velocity (uint16, big-endian)
  [4:5] = torque (uint16, big-endian)
  [6:7] = temperature (uint16, big-endian, unit: 0.1°C)
```

### Type 3 — Enable (host → motor)

```
CAN ID: (0x03 << 24) | (host_id << 8) | motor_id | CAN_EFF_FLAG
Data: 8 bytes of zeros
```

### Type 4 — Stop/Disable (host → motor)

```
CAN ID: (0x04 << 24) | (host_id << 8) | motor_id | CAN_EFF_FLAG
Data: 8 bytes of zeros (normal stop)
       data[0]=1 for fault clear
```

### Type 18 — Parameter Write (host → motor)

```
CAN ID: (0x12 << 24) | (host_id << 8) | motor_id | CAN_EFF_FLAG
Data:
  [0:1] = parameter index (uint16, little-endian)
  [2:3] = 0x00 0x00
  [4:7] = parameter value (float32, little-endian)

Key parameters:
  0x7005 = run mode (0=MIT motion control, 1=position, 2=velocity, 3=current)
```

---

## 8. References

### Primary References

1. **EDULITE_A3** — RobStride official 6-DOF arm ROS2 interface
   https://github.com/RobStride/EDULITE_A3
   Protocol patterns, activation sequence, CAN driver architecture

2. **ros_odrive** — ODrive official ROS2 control interface
   https://github.com/odrive-robotics/ros_odrive
   Architecture patterns, NAN initialization, deferred enable, per-joint struct

### Secondary References

3. **K-Scale actuator** — Rust crate for Robstride motors
   https://github.com/kscalelabs/actuator
   Protocol verification, motor parameter tables

4. **ros2_control documentation** — SystemInterface lifecycle
   https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html

### Tier 1 Humanoid Projects (from references.bib analysis)

5. **K-Scale K-Bot** — 20-DOF humanoid, same Robstride motors
   https://github.com/kscalelabs/kbot
   Validates our motor selection and overall architecture

6. **Fourier GR1/GR2** — Production humanoid with ros2_control
   Uses similar CAN-based motor control architecture

7. **Unitree G1/H1** — Commercial humanoid
   CAN bus motor control with feedback-based activation
