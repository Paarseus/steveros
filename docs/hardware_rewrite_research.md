# SteveROS Hardware Interface Rewrite — Research & Plan

**Date**: 2026-02-24
**Status**: Research complete, ready for implementation
**Prereq**: [docs/hardware_interface_analysis.md](hardware_interface_analysis.md) (root cause analysis)

---

## Phase 1: Research Findings

### 1.1 Robstride Protocol References

**11 open-source projects found** using Robstride RS-series motors. Ranked by relevance to SteveROS:

| # | Repository | Lang | Framework | Motors | Key Value |
|---|-----------|------|-----------|--------|-----------|
| 1 | [RobStride/EDULITE_A3](https://github.com/RobStride/EDULITE_A3) | C++ | ROS2 ros2_control | RS00, EL05 | **Official** Robstride 6-DOF arm, full SystemInterface |
| 2 | [kscalelabs/actuator](https://github.com/kscalelabs/actuator) | Rust | Standalone | RS00-RS04 | Most complete per-model parameter tables |
| 3 | [kscalelabs/kos-kbot](https://github.com/kscalelabs/kos-kbot) | Rust | KOS | RS02-RS04 | 20-DOF humanoid, multi-bus CAN, same form factor |
| 4 | [ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK](https://github.com/ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK) | Python | Standalone | RS00-RS06 | **Official SDK**, authoritative parameter tables |
| 5 | [shdmsrkd/robstride_hardware_interface](https://github.com/shdmsrkd/robstride_hardware_interface) | C++ | ROS2 LifecycleNode | Multiple | ROS2 CAN hardware interface |
| 6 | [liqianqi/rs_motor_bringup](https://github.com/liqianqi/rs_motor_bringup) | C++ | ROS2 ros2_control | Multiple | Another SystemInterface implementation |
| 7 | [RobStride/robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample) | C++ | ROS2 | RS02 | Official ROS2 sample |
| 8 | [odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive) | C++ | ROS2 ros2_control | ODrive | Different protocol, excellent architecture |
| 9 | [sirwart/robstride](https://github.com/sirwart/robstride) | Python | Standalone | Multiple | Python SDK with CLI |
| 10 | [Seeed-Projects/RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control) | Multi | Standalone | RS00-RS06 | Python/C++/Rust/Arduino |
| 11 | [MuShibo/robstride_actuator_bridge](https://github.com/MuShibo/robstride_actuator_bridge) | C++ | ROS1 | RS02 | ROS1 CAN bridge |

#### Motor Activation Sequences Found

**EDULITE_A3 — 3-Step Safe Activation (AUTHORITATIVE)**

This is the most thoroughly engineered sequence found across all 11 projects. Only EDULITE_A3 implements both fault-clearing AND soft-enable (Kp=0).

```
Step 0: CLEAR FAULTS
  for each motor:
    Type 4 frame (stop), data[0]=1  → clear fault flags
    sleep 50ms
  sleep 200ms                       → global settle time

Step 1: SOFT ENABLE (Kp=0)
  for each motor:
    Type 4 frame (stop), data[0]=0  → clean stop
    sleep 30ms
    Type 18 frame, param 0x7005=0   → set run mode to MOTION_CONTROL
    sleep 30ms
    Type 3 frame (enable)           → motor enabled
    Type 1 frame (MIT command):
      position=0, velocity=0,
      Kp=0, Kd=4.0, torque=0       → motor is enabled but exerts NO position force
    sleep 50ms

Step 2: READ INITIAL POSITIONS
  Read from background thread feedback cache (populated since Step 0)

Step 3: POSITION HOLD
  for each motor:
    Type 1 frame (MIT command):
      position=ACTUAL, velocity=0,
      Kp=target, Kd=target, torque=0  → hold at current position
    sleep 30ms
```

**K-Scale actuator — Feedback-Readiness Gate (COMPLEMENTARY)**

Different approach: waits for 5 feedback messages before marking motor as "ready". Control commands only execute when `ready AND enabled` are both true. Validates angle change per command cycle against `max_angle_change` limit.

**shdmsrkd, liqianqi, robstride_ros_sample — Simple Enable (INCOMPLETE)**

These 3 projects call `enable()` directly without fault-clearing or soft-enable. They work for benchtop testing but are unsafe for multi-motor humanoid startup.

#### CAN Driver Architectures

| Feature | EDULITE_A3 | K-Scale actuator | shdmsrkd HW Interface |
|---------|-----------|-----------------|----------------------|
| Receive | Dedicated poll() thread, 10ms timeout | Async Tokio tasks | poll() in read phase |
| Send mutex | std::mutex | TokioMutex on socket | std::mutex |
| Feedback mutex | Separate std::mutex | RwLock on HashMap | command_mutex |
| Send retries | 10 retries, linear backoff (1ms base) | NONE | 3 retries, 500us sleep |
| Send buffer tuning | No | No | Yes (1MB SO_SNDBUF) |

#### Per-Model Motor Parameter Tables (VERIFIED)

Cross-verified across Official SDK, K-Scale actuator crate, and EDULITE_A3 protocol doc:

| Parameter | RS01 | RS02 | RS03 | RS04 |
|-----------|------|------|------|------|
| **Position** (rad) | ±4π | ±4π | ±4π | ±4π |
| **Velocity** (rad/s) | ±44 | ±44 | ±20 | ±15 |
| **Torque** (Nm) | ±17 | ±17 | ±60 | ±120 |
| **Kp** (Nm/rad) | 0–500 | 0–500 | 0–5000 | 0–5000 |
| **Kd** (Nm·s/rad) | 0–5 | 0–5 | 0–100 | 0–100 |

**IMPORTANT**: The current `robstride_protocol.hpp` uses hardcoded ranges (kTorqueMin=-12, kTorqueMax=12, kKpMax=500, kKdMax=5) that are WRONG for RS03 and RS04 motors. This will cause incorrect encoding of commands for the larger actuators in the legs.

**Run Modes** (all models): 0=MOTION_CONTROL (MIT), 1=POSITION_PP, 2=VELOCITY, 3=CURRENT, 5=POSITION_CSP.

#### K-Scale KBot Motor-to-Joint Mapping (Reference for SteveROS)

| Group | Motor IDs | Motor Type |
|-------|-----------|------------|
| Left arm | 11-12 | RS03 |
| Left arm | 13-15 | RS02 |
| Right arm | 21-22 | RS03 |
| Right arm | 23-25 | RS02 |
| Left leg | 31, 34 | RS04 |
| Left leg | 32-33 | RS03 |
| Left leg | 35 | RS02 |
| Right leg | 41, 44 | RS04 |
| Right leg | 42-43 | RS03 |
| Right leg | 45 | RS02 |

---

### 1.2 ros2_control Best Practices

#### Lifecycle Responsibility Consensus (6 implementations agree)

| Phase | Responsibility |
|-------|---------------|
| `on_init()` | Parse parameters, validate config, allocate memory. NO hardware communication. |
| `on_configure()` | Establish communication (CAN, serial, socket). Create ROS2 publishers/services. |
| `on_activate()` | Read current hardware state, sync commands to state, enable motor power. |
| `on_deactivate()` | Disable motors, set to idle/safe state. |
| `on_cleanup()` | Tear down communication channels. |

#### NAN Initialization Pattern (5/6 implementations agree)

The ros2_control `Handle` class initializes all double state values to NAN by default:

```cpp
// From hardware_interface/handle.hpp
value_ = initial_value.empty() ? std::numeric_limits<double>::quiet_NaN()
                               : hardware_interface::stod(initial_value);
```

| Implementation | States init to NAN? | Where? |
|---|---|---|
| ros2_control framework (Handle) | **YES** | Constructor |
| ros_odrive | **YES** | Struct member declaration |
| Reachy2 / Orbita3D | **YES** | `on_activate()` |
| Reachy2 Dynamixel | **YES** | `on_activate()` |
| gz_ros2_control | **YES** | `initSim()` |
| ros2_control_demos example_7 | NO | Uses 0.0 (simple demo) |

**Convention**: Commands default to 0.0 (safe zero), States default to NAN (unknown until hardware reports).

#### Commands Synced to Current State Before Enabling (ALL real-hardware impls)

- **ros_odrive**: Motor enabled only in `perform_command_mode_switch()` (deferred)
- **Orbita3D**: `hw_commands_position_ = hw_states_position_` in `on_activate()`
- **Reachy2 Dynamixel**: `hw_commands_position_ = hw_states_position_` in `on_activate()`

This prevents the snap-to-zero bug.

#### perform_command_mode_switch() vs on_activate()

**ros_odrive** key comment: *"This can be called several seconds before the controller finishes starting. Therefore we enable the ODrives only in `perform_command_mode_switch()`."*

The two-phase pattern:
1. `prepare_command_mode_switch()` — validate the requested mode change (can reject)
2. `perform_command_mode_switch()` — execute the actual mode change

**For SteveROS**: Our activation sequence is complex (4 seconds for fault-clear + soft-enable). Doing this in `on_activate()` is appropriate because we need guaranteed motor state before the first `read()` cycle. The ODrive defers because its enable is a single CAN frame; our multi-step sequence benefits from the explicit lifecycle callback.

#### Controller-Side NAN Guards

`JointTrajectoryController` checks for NAN states:
```cpp
auto interface_has_values = [](const auto & joint_interface) {
    return std::find_if(...,
        [](const auto & interface) {
            auto interface_op = interface.get().get_optional();
            return !interface_op.has_value() || std::isnan(interface_op.value());
        }) == joint_interface.end();
};
```

`forward_command_controller` validates with `std::isfinite()`.

**Implication**: Returning NAN from state interfaces is safe — controllers will wait for valid data.

#### Single SystemInterface for Multi-DOF Robots

TALOS (30+ DOF) uses a single `ros2_control_talos_system` for ALL joints. This matches our plan for a single SteveROSHardware managing all 20 motors on a shared CAN bus.

---

### 1.3 CAN Bus Multi-Motor Patterns

#### Background Receive Thread: poll() vs epoll vs select

| Factor | poll() | epoll | select() |
|--------|--------|-------|----------|
| Single-FD overhead | Negligible | Marginally more setup | Negligible |
| Multi-FD scaling | O(n) | O(1) | O(n), limited to FD_SETSIZE |
| Code complexity | Low | Medium | Low |
| Reference match | EDULITE_A3 (same protocol) | ros_odrive | Autoware |
| SteveROS FD count | 1 | 1 | 1 |

**Winner: poll()** — identical performance to epoll for a single FD, simpler, proven with the same Robstride protocol.

EDULITE_A3 pattern:
```
receiveThread():
  while (running_):
    struct pollfd pfd = {socket_, POLLIN, 0}
    if poll(&pfd, 1, 10ms) > 0:     ← 10ms timeout
      read frame from socket
      parse feedback
      lock(feedback_mutex_)
      feedback_cache_[motor_id] = feedback
      unlock()
```

#### CAN Bus Timing Analysis (20 Motors @ 1 Mbps)

Extended CAN frame: ~148 bits typical (131 bits + bit stuffing).
At 1 Mbps: **~148 μs per frame**.

| Phase | Frames | Time (typical) | Time (worst) |
|-------|--------|----------------|--------------|
| Command TX (20 motors) | 20 | 2,960 μs | 3,100 μs |
| Feedback RX (20 motors) | 20 | 2,960 μs | 3,100 μs |
| Processing overhead | — | ~500 μs | ~1,000 μs |
| **Total** | **40** | **~6,400 μs** | **~7,200 μs** |
| **Remaining in 10ms budget** | — | **~3,600 μs** | **~2,800 μs** |

**Bus utilization**: ~59% — well under the 80% design rule. 100 Hz is feasible.

#### Activation Timing (20 Motors, Sequential)

| Phase | Per-Motor | Total (20 motors) |
|-------|-----------|-------------------|
| Clear faults | 50ms + 200ms settle | 1,200 ms |
| Soft enable | 110ms (30+30+50) | 2,200 ms |
| Position hold | 30ms | 600 ms |
| **Total** | — | **~4,000 ms** |

4 seconds for startup is acceptable (one-time cost).

#### SocketCAN Configuration Requirements

| Setting | Value | Rationale |
|---------|-------|-----------|
| txqueuelen | 128 | Default 10 causes ENOBUFS with 20 motors per cycle |
| SOCK_RAW | Yes | Standard for raw CAN frames |
| CAN_RAW_FILTER | Type 2 only | Kernel-level filtering, reduce userspace load |
| SO_RCVTIMEO | 100ms | Safety backstop (matches EDULITE_A3) |
| Non-blocking | No | Use poll() with timeout instead |

**System config required**: `ip link set can0 txqueuelen 128`

---

### 1.4 Cross-Reference Agreement/Disagreement Matrix

| Topic | Finding | Confidence | Agreeing Sources |
|-------|---------|------------|-----------------|
| **States → NAN** | Initialize states to NAN, not 0.0 | **HIGH** | ros2_control framework, ros_odrive, Reachy2, Orbita3D, gz_ros2_control |
| **Sync commands to state** | Set commands = current state before enabling | **HIGH** | ros_odrive, Orbita3D, Reachy2 Dynamixel |
| **Fault clear first** | Send Type 4 data[0]=1 before enabling | **HIGH** | EDULITE_A3, Official protocol doc |
| **Soft enable (Kp=0)** | Enable with Kp=0 before position hold | **HIGH** | EDULITE_A3 (only impl, but official) |
| **Background receive thread** | poll() in dedicated thread | **HIGH** | EDULITE_A3, shdmsrkd |
| **Separate send/feedback mutex** | Two mutexes, not one | **HIGH** | EDULITE_A3, K-Scale actuator |
| **Send retries for ENOBUFS** | 3-10 retries with backoff | **HIGH** | EDULITE_A3 (10), shdmsrkd (3), libcsp |
| **Sequential activation** | One motor at a time with delays | **HIGH** | EDULITE_A3, all Robstride projects |
| **Single SystemInterface** | One plugin for all joints | **HIGH** | TALOS, ros_odrive, all reference impls |
| **Per-model param tables** | RS03/RS04 have different ranges than RS01/RS02 | **HIGH** | Official SDK, K-Scale actuator, EDULITE_A3 |
| **Set run mode before enable** | Type 18, param 0x7005=0 | **MEDIUM** | EDULITE_A3 (only ROS2 impl that does it) |
| **50ms inter-command delay** | During activation phases | **MEDIUM** | EDULITE_A3 (may be conservative) |
| **200ms post-fault settle** | After clearing all faults | **MEDIUM** | EDULITE_A3 (may be conservative) |
| **Feedback readiness gate** | Wait for N feedbacks before commanding | **MEDIUM** | K-Scale actuator (5 msgs) |
| **SCHED_FIFO for receive thread** | Real-time priority ~49 | **LOW** | General Linux RT practice, no Robstride evidence |

**Key Disagreements**:

| Topic | View A | View B | Resolution |
|-------|--------|--------|------------|
| Motor enable timing | on_activate() (EDULITE_A3, Reachy2) | perform_command_mode_switch() (ros_odrive) | Use on_activate() — our sequence is multi-step and needs guaranteed completion |
| Send retry count | 10 (EDULITE_A3) | 3 (shdmsrkd) | Use 10 — conservative is better for 20-motor bus |
| Receive pattern | Background thread (EDULITE_A3) | In-loop drain (current SteveROS) | Background thread — prevents frame loss between read() cycles |

---

## Phase 2: Architecture Decisions

### ADR-1: Threading Model

**Question**: Single-threaded blocking vs background receive thread vs epoll event loop?

**Options**:

| Option | Evidence | Pros | Cons |
|--------|----------|------|------|
| A. Single-threaded (current) | Current SteveROS | Simple | Loses frames between read() calls, 10ms gap |
| B. Background thread + poll() | EDULITE_A3, shdmsrkd | Proven with Robstride, continuous reception | Extra thread, mutex needed |
| C. epoll event loop | ros_odrive | Event-driven, no extra thread | More complex, no benefit for single FD |

**Decision: B — Background receive thread with poll()** [HIGH confidence]

**Rationale**: With 20 motors at 100 Hz, feedback frames arrive continuously. The current in-loop drain (option A) has a 10ms gap between `read()` calls where frames could pile up or overflow the kernel buffer. A background thread with `poll(fd, 1, 10ms)` provides continuous reception with minimal overhead. This is the exact pattern proven by EDULITE_A3 with the same Robstride protocol.

**Specifics**: `poll()` with 10ms timeout, `std::thread` with `std::atomic<bool>` stop flag, thread started in `on_configure()`, stopped in `on_cleanup()`.

**Open question**: Whether to set `SCHED_FIFO` priority. Not needed for initial testing; revisit if frame drops are observed in production.

---

### ADR-2: Activation Sequence

**Question**: What is the exact, validated CAN frame sequence for safe motor startup?

**Decision: Adopt EDULITE_A3 3-step sequence with feedback validation** [HIGH confidence]

```
Phase 0: CLEAR FAULTS (in on_activate)
  for each motor (i = 0..N-1):
    send Type 4 frame, data[0] = 1           // fault-clear stop
    usleep(50'000)                            // 50ms per motor
  usleep(200'000)                             // 200ms global settle

Phase 1: SOFT ENABLE (in on_activate)
  for each motor:
    send Type 18 frame, param 0x7005 = 0      // set run mode = MIT
    usleep(30'000)
    send Type 3 frame                          // enable motor
    send Type 1 frame:                         // MIT command: Kp=0 (limp)
      p_des=0, v_des=0, kp=0, kd=Kd, t_ff=0
    usleep(50'000)

Phase 2: READ INITIAL POSITIONS (in on_activate)
  for each motor:
    fb = feedback_cache[motor_id]              // from background thread
    if fb.is_valid:
      joint.position = motor_to_joint(fb.position)
      joint.command = joint.position           // SYNC: prevents snap-to-zero
    else:
      RCLCPP_ERROR("No feedback from motor %d after activation")
      return ERROR

Phase 3: START Kp RAMP (begins in first write() cycle)
  activation_time_ = now
  ramping_ = true
  // Kp ramps 0 → target over kp_ramp_duration_s_ (default 1.0s)
  // But commanding the ACTUAL position, not 0.0
```

**Timing for 20 motors**: ~4 seconds total (acceptable one-time startup cost).

**Failure handling**:
- If any motor has no feedback after Phase 1 + 50ms wait → return ERROR from on_activate()
- If fault flags are set in feedback after Phase 1 → log and return ERROR
- Controller manager will handle the error by not transitioning to ACTIVE state

**Validated against**: EDULITE_A3 (exact protocol) and K-Scale actuator (readiness gate concept).

**Open questions**:
1. Can the 50ms inter-motor delay in Phase 0 be reduced? (needs hardware testing)
2. Is the 200ms post-fault settle necessary for motors not in fault state? (needs testing)
3. Should we verify motor mode == 2 (Run) in feedback after Phase 1? (likely yes)

---

### ADR-3: State Initialization

**Question**: NAN vs 0.0 vs "pending" flag?

**Decision: NAN-initialized states** [HIGH confidence]

**Rationale**: 5 out of 6 reference implementations use NAN for uninitialized states. The ros2_control `Handle` class defaults to NAN. Controllers (`JointTrajectoryController`, `forward_command_controller`) explicitly check for NAN and wait for valid data. This is the canonical pattern.

**Implementation**:
```cpp
struct JointState {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort   = std::numeric_limits<double>::quiet_NaN();
    double command_position = 0.0;  // Commands default to 0.0 (safe zero)
};
```

States transition from NAN → valid when the first feedback from that motor is received and decoded in the background thread.

Commands are synced to actual position in `on_activate()` Phase 2 (before the first `write()` cycle).

**No open questions** — this is the most strongly supported decision in the entire research.

---

### ADR-4: Layer Separation

**Question**: What goes in each layer?

**Decision: Three-layer architecture** [HIGH confidence]

```
┌─────────────────────────────────────────────────────┐
│  ros2_control Controller Manager (100 Hz)           │
│  ┌─────────────────────────────────────────────────┐│
│  │  JointTrajectoryController                      ││
│  │  (interpolation, trajectory execution)          ││
│  └────────────────┬────────────────────────────────┘│
│                   │ position commands                │
│  ┌────────────────▼────────────────────────────────┐│
│  │  SteveROSHardware (SystemInterface)             ││
│  │  steveros_hardware.hpp / .cpp                   ││
│  │  ─────────────────────────────                  ││
│  │  • Lifecycle (on_init/configure/activate/...)   ││
│  │  • State/command interface export               ││
│  │  • Coordinate transforms (sign, offset)         ││
│  │  • Kp ramp on activation                        ││
│  │  • Feedback timeout checking                    ││
│  │  • URDF parameter parsing                       ││
│  │  • Per-joint JointState structs (NAN-init)      ││
│  └────────────────┬────────────────────────────────┘│
│                   │ motor commands (motor-space)     │
│  ┌────────────────▼────────────────────────────────┐│
│  │  RobstrideCanDriver                             ││
│  │  robstride_can_driver.hpp / .cpp                ││
│  │  ─────────────────────────────                  ││
│  │  • SocketCAN open/close                         ││
│  │  • Background receive thread (poll)             ││
│  │  • send_frame() with retries + send_mutex_      ││
│  │  • Thread-safe feedback cache + feedback_mutex_ ││
│  │  • Motor enable/disable/fault-clear sequences   ││
│  │  • Per-model MotorParams lookup                 ││
│  │  • Motor type → encoding range mapping          ││
│  └────────────────┬────────────────────────────────┘│
│                   │ raw CAN frames                   │
│  ┌────────────────▼────────────────────────────────┐│
│  │  robstride_protocol.hpp (header-only)           ││
│  │  ─────────────────────────────                  ││
│  │  • MIT command encoding (Type 1)                ││
│  │  • Feedback decoding (Type 2)                   ││
│  │  • Enable/Stop encoding (Type 3/4)              ││
│  │  • Param write encoding (Type 18)               ││
│  │  • MotorParams struct (per-model ranges)        ││
│  │  • float↔uint16 conversion with model ranges   ││
│  └────────────────┬────────────────────────────────┘│
│                   │                                  │
│              SocketCAN (can0)                        │
└─────────────────────────────────────────────────────┘
```

**Boundary rules**:

| Responsibility | Where it lives | NOT here |
|---------------|---------------|----------|
| CAN frame byte layout | robstride_protocol.hpp | — |
| Per-model encoding ranges | robstride_protocol.hpp | — |
| Socket management, threading | RobstrideCanDriver | Not in SteveROSHardware |
| Send retries, mutexes | RobstrideCanDriver | Not in SteveROSHardware |
| Motor enable/disable sequence | RobstrideCanDriver | — |
| URDF parsing, lifecycle | SteveROSHardware | Not in driver |
| Coordinate transforms | SteveROSHardware | Not in driver |
| Kp ramp, feedback timeout | SteveROSHardware | Not in driver |
| Gravity compensation | Controller plugin | NOT in hardware interface |
| Trajectory smoothing | JointTrajectoryController | NOT in hardware interface |
| Debug topic publishing | ros2_control state_broadcaster | NOT in hardware interface |
| Velocity filtering | Controller plugin | NOT in hardware interface |

---

### ADR-5: Per-Model Parameters

**Question**: Hardcoded lookup table vs URDF params vs config file?

**Options**:

| Option | Evidence | Pros | Cons |
|--------|----------|------|------|
| A. Hardcoded constexpr table | EDULITE_A3, K-Scale actuator | Simple, zero config, compile-time checked | Requires recompile for new motors |
| B. URDF params per joint | liqianqi rs_motor_bringup | Flexible per-joint | Verbose URDF, easy to misconfigure |
| C. YAML config file | General ROS2 practice | Decoupled from code | Extra file, runtime parsing |

**Decision: A — Hardcoded constexpr lookup table in robstride_protocol.hpp** [HIGH confidence]

**Rationale**:
- Motor parameter ranges are fixed by motor hardware — they never change at runtime
- Both EDULITE_A3 and K-Scale actuator use compile-time tables
- A `motor_type` URDF param per joint selects from the table (RS01/RS02/RS03/RS04)
- The URDF only specifies `motor_type` (enum), not raw ranges — preventing misconfiguration
- Adding a new motor model requires adding one table entry and recompiling

**Implementation**: Add `MotorParams` struct and `get_motor_params(MotorType)` to `robstride_protocol.hpp`:

```
enum class MotorType { RS01, RS02, RS03, RS04 };

struct MotorParams {
    float pos_min, pos_max;     // always ±4π
    float vel_min, vel_max;     // model-specific
    float torque_min, torque_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
};

constexpr MotorParams get_motor_params(MotorType type);
```

URDF change: Add `<param name="motor_type">RS03</param>` to each joint.

---

### ADR-6: Send Retry Policy

**Question**: How many retries? What backoff strategy? What happens on permanent failure?

**Decision: 10 retries, linear backoff, 500μs base** [HIGH confidence]

| Parameter | Value | Source |
|-----------|-------|--------|
| Max retries | 10 | EDULITE_A3 |
| Base delay | 500 μs | Tuned down from EDULITE_A3's 1ms for 20-motor timing |
| Backoff | Linear: delay = 500μs × (retry + 1) | EDULITE_A3 pattern |
| Worst-case total delay | 27.5 ms (500+1000+...+5000 μs) | Calculated |
| Retryable errors | ENOBUFS, EAGAIN, EWOULDBLOCK | EDULITE_A3, kernel docs |
| Non-retryable errors | All others → immediate fail | EDULITE_A3 |
| Serialization | send_mutex_ (std::mutex) | EDULITE_A3 |

**On permanent failure**: `send_frame()` returns false. The caller (`write()`) logs a warning but does NOT return ERROR — one dropped command frame is tolerable at 100 Hz. Multiple consecutive failures should trigger a warning counter, but a single missed frame is not worth deactivating the entire robot.

**System prerequisite**: `ip link set can0 txqueuelen 128` to reduce ENOBUFS frequency. Document this in the launch/bringup README.

---

## Phase 3: Rewrite Plan

### 3.1 File Inventory

| # | File Path | Status | Est. Lines | Purpose |
|---|-----------|--------|-----------|---------|
| 1 | `include/.../robstride_protocol.hpp` | **EXTEND** | 230 (+42) | Add MotorType enum, MotorParams struct, per-model lookup table, Type 18 encode, fault-clear encode |
| 2 | `include/.../robstride_can_driver.hpp` | **NEW** | 100 | RobstrideCanDriver class declaration: socket, threads, mutexes, feedback cache, public API |
| 3 | `src/robstride_can_driver.cpp` | **NEW** | 300 | CAN driver implementation: open/close, background thread, send with retries, enable/disable/fault-clear sequences |
| 4 | `include/.../steveros_hardware.hpp` | **REWRITE** | 80 | SteveROSHardware class: JointState struct (NAN-init), per-joint vector, driver pointer, lifecycle methods |
| 5 | `src/steveros_hardware.cpp` | **REWRITE** | 280 | SystemInterface implementation: lifecycle, interface export, read/write, coordinate transforms, Kp ramp |
| 6 | `CMakeLists.txt` | **EDIT** | +3 | Add `src/robstride_can_driver.cpp` to sources |
| 7 | `steveros_hardware.xml` | **KEEP** | 7 | Plugin descriptor (unchanged) |
| 8 | `include/.../visibility_control.h` | **KEEP** | — | Unchanged |

**Estimated total: ~990 lines** (protocol 230 + driver header 100 + driver impl 300 + hw header 80 + hw impl 280)

This is ~15% larger than the current 802 lines but includes a proper CAN driver layer that currently doesn't exist. The hardware interface itself (steveros_hardware.hpp + .cpp) shrinks from 614 → 360 lines by delegating CAN I/O to the driver.

### File Details

#### File 1: `robstride_protocol.hpp` (EXTEND, ~230 lines)

**Keep everything currently in the file** — all encode/decode functions are correct and validated.

**Add**:
- `enum class MotorType { RS01, RS02, RS03, RS04 }` — motor model enum
- `struct MotorParams { float pos_min, pos_max, vel_min, vel_max, ... }` — per-model ranges
- `constexpr MotorParams get_motor_params(MotorType)` — lookup function
- `MotorType parse_motor_type(const std::string&)` — string-to-enum for URDF parsing
- `can_frame encode_stop_clear_fault(int motor_id)` — Type 4 with data[0]=1
- `can_frame encode_param_write(int motor_id, uint16_t index, float value)` — Type 18
- Update `encode_mit_command()` to accept `MotorParams` for per-model encoding ranges
- `static constexpr uint32_t kTypeParamWrite = 18` — add missing type constant

**Dependencies**: None (header-only, standard library only).

#### File 2: `robstride_can_driver.hpp` (NEW, ~100 lines)

**Purpose**: Class declaration for the CAN bus driver layer.

**Contents**:
```
class RobstrideCanDriver {
public:
    bool open(const std::string& interface);
    void close();

    bool send_frame(const can_frame& frame);          // With retries + mutex
    MotorFeedback get_feedback(int motor_id) const;    // Thread-safe copy from cache
    bool has_valid_feedback(int motor_id) const;       // Check if feedback received

    // Motor control sequences
    bool clear_fault(int motor_id);
    bool set_run_mode(int motor_id, uint8_t mode);
    bool enable_motor(int motor_id);
    bool disable_motor(int motor_id);

private:
    int socket_fd_ = -1;
    std::thread receive_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex send_mutex_;
    mutable std::mutex feedback_mutex_;
    std::unordered_map<int, MotorFeedback> feedback_cache_;

    void receive_loop();                               // Background thread function
};
```

**Dependencies**: `robstride_protocol.hpp`, `<linux/can.h>`, standard library.

#### File 3: `robstride_can_driver.cpp` (NEW, ~300 lines)

**Purpose**: CAN driver implementation.

**Key functions**:
- `open()` — create socket, bind to interface, set CAN filter, set SO_RCVTIMEO, start receive thread (~40 lines)
- `close()` — stop thread, join, close socket (~15 lines)
- `send_frame()` — lock send_mutex_, write with 10 retries and linear backoff (~25 lines)
- `receive_loop()` — poll() with 10ms timeout, decode feedback, update cache under feedback_mutex_ (~35 lines)
- `get_feedback()` — lock feedback_mutex_, return copy (~10 lines)
- `has_valid_feedback()` — lock, check existence (~8 lines)
- `clear_fault()` — encode_stop_clear_fault + send_frame (~5 lines)
- `set_run_mode()` — encode_param_write(0x7005, mode) + send_frame (~5 lines)
- `enable_motor()` — encode_enable + send_frame (~5 lines)
- `disable_motor()` — encode_stop + send_frame (~5 lines)

**Dependencies**: `robstride_protocol.hpp`, `robstride_can_driver.hpp`.

#### File 4: `steveros_hardware.hpp` (REWRITE, ~80 lines)

**Purpose**: Hardware interface class declaration with NAN-initialized JointState.

**Key changes from current**:
- Replace 6 parallel vectors with a single `std::vector<JointState>` (per-joint struct)
- `JointState` has NAN-initialized state fields (position, velocity, effort)
- Add `MotorType motor_type` to JointConfig (parsed from URDF)
- Replace `int can_socket_` with `std::unique_ptr<RobstrideCanDriver> driver_`
- Remove raw socket includes — those move to the driver

**Dependencies**: `robstride_can_driver.hpp`, ros2_control headers.

#### File 5: `steveros_hardware.cpp` (REWRITE, ~280 lines)

**Purpose**: SystemInterface lifecycle implementation.

**Key functions**:
- `on_init()` — parse URDF params including motor_type, build motor_id map, validate interfaces (~80 lines, mostly preserved from current)
- `on_configure()` — `driver_->open(can_interface_)` (~15 lines, much simpler)
- `on_activate()` — 3-phase activation sequence using driver API, read feedback, sync commands (~60 lines)
- `on_deactivate()` — zero-torque + disable via driver (~15 lines, preserved)
- `on_cleanup()` — `driver_->close()` (~8 lines)
- `export_state_interfaces()` / `export_command_interfaces()` — iterate JointState vector (~25 lines)
- `read()` — copy from driver feedback cache, apply transforms, check timeout (~40 lines)
- `write()` — encode commands with per-model params, Kp ramp, send via driver (~35 lines)
- `motor_to_joint()` / `joint_to_motor()` — coordinate transforms (~10 lines, preserved)

**Dependencies**: `steveros_hardware.hpp`, `robstride_can_driver.hpp`, `robstride_protocol.hpp`.

---

### 3.2 Build Order

**Step 1 — Extend `robstride_protocol.hpp`** (no dependencies)
- Add MotorType, MotorParams, lookup table
- Add Type 18 and fault-clear frame encoders
- Update encode_mit_command() to accept MotorParams
- Verify: compile-only (header changes)

**Step 2 — Implement `RobstrideCanDriver`** (depends on Step 1)
- Write header + implementation
- Test: standalone CAN send/receive test (no ros2_control needed)
- Smoke test: open can0, send enable to 1 motor, verify feedback cache populates

**Step 3 — Rewrite `SteveROSHardware`** (depends on Steps 1 & 2)
- Write new header with JointState struct
- Write implementation using driver API
- Test: single-motor activation via ros2_control launch

**Step 4 — Integration test**
- Test with 1 motor: verify no snap-to-zero
- Test with 3 motors: verify multi-motor timing
- Test with 20 motors: verify bus timing and feedback reliability

---

### 3.3 Smoke Test Plan

**Test 0: Protocol Layer** (no hardware needed)
```
Compile the extended robstride_protocol.hpp.
Verify MotorParams lookup returns correct ranges for all 4 types.
Verify encode_param_write() produces correct CAN frame bytes.
Verify encode_stop_clear_fault() produces correct CAN frame bytes.
Unit test: encode_mit_command() with RS03 MotorParams uses Kp range 0-5000 (not 0-500).
```

**Test 1: CAN Driver** (1 motor on bench)
```
Create a standalone test node that:
1. Opens the CAN driver on can0
2. Calls clear_fault(motor_id)
3. Waits 500ms
4. Calls set_run_mode(motor_id, 0)
5. Calls enable_motor(motor_id)
6. Sends 10x MIT command with Kp=0 (motor should be limp)
7. Reads feedback cache — verify position is non-zero and plausible
8. Sends MIT command with Kp=20, position=current_position
9. Motor should hold position gently
10. Calls disable_motor(motor_id)
```

**Test 2: Hardware Interface** (1 motor via ros2_control)
```
Launch with only 1 joint configured in URDF.
1. `ros2 control load_controller --set-state configure joint_trajectory_controller`
2. `ros2 control set_hardware_component_state steveros_hardware active`
3. Verify motor does NOT snap to zero (the critical bug fix)
4. `ros2 topic echo /joint_states` — verify position is plausible
5. Send a small position command via action client
6. Verify smooth motion
7. Deactivate — verify clean shutdown
```

**Test 3: Multi-Motor** (3 motors, then 20)
```
Repeat Test 2 with 3 joints, then all 20 joints.
Monitor: CAN bus utilization, feedback timeout rate, send retry rate.
Verify: activation completes in <5 seconds, no motors snap to zero.
```

---

### 3.4 Exclusion List

The following features are explicitly **EXCLUDED** from the hardware interface. They belong in controller plugins or external nodes:

| Feature | Where it belongs | Rationale |
|---------|-----------------|-----------|
| Gravity compensation (Pinocchio RNEA) | Custom controller plugin | EDULITE_A3 puts this in hardware — wrong layer |
| S-curve trajectory smoothing | JointTrajectoryController built-in | Controller responsibility, not I/O |
| Velocity IIR filtering | Custom controller plugin | Signal processing, not I/O |
| Debug topic publishers | ros2_control `joint_state_broadcaster` | Built-in to ros2_control framework |
| Zero-torque teach mode | Service in a separate node | Application-level feature |
| YAML inertia calibration | Controller plugin or config | Not hardware I/O |
| Position/velocity limits enforcement | URDF `<limit>` + controller | ros2_control handles this |
| Multi-CAN-bus support | Future: pass interface name per joint group | Current: single can0 |

---

### 3.5 Open Questions (Ranked by Impact)

| Rank | Question | Impact | How to Resolve | Confidence Without Testing |
|------|----------|--------|---------------|---------------------------|
| 1 | **Can the 50ms inter-motor fault-clear delay be reduced?** | Activation time: 1s vs 4s for 20 motors | Send all fault-clears back-to-back (3ms wire time), wait 200ms total, check feedback for all motors | MEDIUM — probably safe to batch, but EDULITE_A3 is conservative for a reason |
| 2 | **Do RS03/RS04 motors require different Kd defaults during soft-enable?** | Kd range is 0-100 for RS03/RS04 vs 0-5 for RS01/RS02. A Kd of 4.0 might be too high or too low. | Test soft-enable with Kd=4.0 on RS03/RS04. If too stiff, try Kd=1.0. If too loose, try Kd=10.0. | LOW — no reference data for RS03/RS04 soft-enable gains |
| 3 | **Is Type 18 setRunMode required before every enable, or only on first boot?** | Could skip this step and save 30ms × 20 motors = 600ms | Enable without setRunMode on a motor that was previously in MIT mode. If it works, the step is only needed on first enable after power-on. | MEDIUM — EDULITE_A3 always sends it, but this may be defensive |
| 4 | **What is the minimum time between enable (Type 3) and first feedback?** | Determines how long to wait in Phase 2 before reading feedback | Send enable, log timestamp of first feedback frame. Repeat 10 times, take max. Expected: <10ms based on CAN timing. | HIGH — probably fast, but need to measure |
| 5 | **Does motor mode == 2 (Run) always appear in feedback after successful enable?** | Could use this as a health check in on_activate() | Check feedback mode field after enable sequence. If mode != 2, motor did not activate properly. | MEDIUM — protocol doc says mode 2 = Run, but untested |
| 6 | **What txqueuelen value prevents ENOBUFS without excessive buffering?** | txqueuelen 128 is a guess; 64 might suffice, 256 might be needed | Run 20-motor write loop at 100 Hz, count ENOBUFS events with txqueuelen = 32, 64, 128, 256 | MEDIUM — 128 is a safe estimate |
| 7 | **Is the CAN receive filter correct for all Robstride response types?** | Current filter accepts only Type 2. Type 21 (fault) frames would be filtered out. | Check if motors send Type 21 fault frames during normal operation. If so, expand filter to accept Type 2 and Type 21. | MEDIUM — current SteveROS only uses Type 2, but fault frames could be useful |
| 8 | **What happens if a motor is already enabled when clear_fault is sent?** | Could the motor jerk or lose position? | Test: enable motor, hold position, send clear_fault, observe behavior. | LOW — EDULITE_A3 always disables before clearing, but edge case matters for re-activation |

---

## Appendix A: CAN Protocol Quick Reference

### Frame Types Used by SteveROS

| Type | Dir | Purpose | CAN ID Bits [28:24] | Data |
|------|-----|---------|---------------------|------|
| 1 | Host→Motor | MIT command | 0x01 | pos[0:1], vel[2:3], Kp[4:5], Kd[6:7]; t_ff in ID[23:8] |
| 2 | Motor→Host | Feedback | 0x02 | pos[0:1], vel[2:3], torque[4:5], temp[6:7]; motor_id in ID[15:8] |
| 3 | Host→Motor | Enable | 0x03 | 8 bytes zeros |
| 4 | Host→Motor | Stop/Fault-clear | 0x04 | data[0]=0 (stop) or data[0]=1 (clear fault) |
| 18 | Host→Motor | Param write | 0x12 | index[0:1] LE, pad[2:3], value[4:7] LE float |

### CAN ID Structure (29-bit extended)

```
Bits [28:24]: Communication type (5 bits)
Bits [23:8]:  Data area 2 / Host CAN ID (16 bits)
Bits [7:0]:   Target motor CAN ID (8 bits)
```

### Key Parameters (Type 18)

| Index | Name | Type | Range |
|-------|------|------|-------|
| 0x7005 | Run mode | uint8 | 0=MIT, 1=Position, 2=Velocity, 3=Current |
| 0x7006 | Iq reference | float | Model-specific |
| 0x700A | Speed reference | float | Model-specific |
| 0x700B | Torque limit | float | Model-specific |

### Fault Flags (Feedback CAN ID bits [21:16])

| Bit | Fault |
|-----|-------|
| 16 | Under-voltage |
| 17 | Phase over-current |
| 18 | Over-temperature |
| 19 | Encoder fault |
| 20 | Jam/stall |
| 21 | Uncalibrated |

---

## Appendix B: Source References

### Primary (Exact Protocol Match)
1. [RobStride/EDULITE_A3](https://github.com/RobStride/EDULITE_A3) — Official 6-DOF arm
2. [kscalelabs/actuator](https://github.com/kscalelabs/actuator) — Rust Robstride protocol
3. [kscalelabs/kos-kbot](https://github.com/kscalelabs/kos-kbot) — 20-DOF humanoid (same form factor)
4. [ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK](https://github.com/ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK) — Official SDK

### Architecture (ros2_control Patterns)
5. [odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive) — NAN-init, per-joint struct, deferred enable
6. [pollen-robotics/orbita3d_control](https://github.com/pollen-robotics/orbita3d_control) — Rust FFI + NAN-init pattern
7. [pollen-robotics/reachy2_core](https://github.com/pollen-robotics/reachy2_core) — Dynamixel NAN-init
8. [pal-robotics/talos_robot](https://github.com/pal-robotics/talos_robot) — 30+ DOF single SystemInterface
9. [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) — Framework Handle NAN default
10. [ros-controls/ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) — Lifecycle patterns
11. [ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) — Simulator interface
12. [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) — Gazebo NAN sentinel

### CAN Bus Patterns
13. [shdmsrkd/robstride_hardware_interface](https://github.com/shdmsrkd/robstride_hardware_interface) — SO_SNDBUF tuning, 3-retry send
14. [autowarefoundation/ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) — select() pattern
15. [libcsp/libcsp](https://github.com/libcsp/libcsp) — ENOBUFS retry, txqueuelen guidance
