# Zero-Offset Calibration Report for SteveROS

## 1. Executive Summary

SteveROS is a 20-DOF humanoid robot using Robstride RS-series CAN bus motors with a ros2_control hardware interface. The core calibration problem is straightforward: each motor's internal encoder zero is at an arbitrary position relative to the URDF-defined joint zero. When the robot powers on, RViz shows incorrect joint angles because all `zero_offset_deg` parameters are set to 0. The offset infrastructure already exists in the hardware interface (`motor_to_joint` / `joint_to_motor` transforms at `steveros_hardware.cpp:460-468`) — what's missing is the actual measured offset values for each of the 20 joints.

**Recommended approach:** Use the existing software-level offset infrastructure with a manual calibration procedure. Physically position the robot in its URDF zero pose, read the raw motor encoder positions via `ros2 topic echo`, and enter the measured values as `zero_offset_deg` parameters in `steveros_ros2_control.xacro`. This requires zero code changes and can be completed in under an hour. For long-term robustness, add Robstride firmware-level zeroing (Type 6 CAN command) as a one-time motor provisioning step, keeping software offsets for fine-tuning.

The current SteveROS approach of handling offsets in the hardware interface (rather than the ros2_control transmission layer) is pragmatically sound for direct-drive 1:1 motors. This pattern is validated by ROBOTO ORIGIN, K-Scale KBot, and multiple community projects. The canonical ros2_control `SimpleTransmission` offset mechanism exists but adds complexity without benefit for 1:1 direct-drive motors.

---

## 2. Current SteveROS Implementation Analysis

### 2.1 Data Flow Diagram

```
MOTOR ENCODER TO JOINT STATE (Read Path):
==========================================

  Motor Physical Position
       |
       v
  CAN Feedback Frame (Type 2, 29-bit extended ID)
  +-- Position: 16-bit raw uint [0, 65535]
  +-- Velocity: 16-bit raw uint
  +-- Torque:   16-bit raw uint
       |
       v
  robstride_can_driver.cpp: receive_loop()
  +-- Extract motor_id from CAN arb_id          [line 143]
  +-- Look up motor_type from motor_types_ map   [lines 146-152]
  +-- Call decode_feedback(frame, params)         [line 155]
       |
       v
  robstride_protocol.hpp: decode_feedback()
  +-- pos_raw -> fb.position                     [line 283]
  |   uint_to_float(pos_raw, -4pi, +4pi, 16)
  +-- vel_raw -> fb.velocity                     [line 284]
  +-- torque_raw -> fb.torque                    [line 285]
       |
       v
  feedback_cache_[motor_id] = fb                 [line 162]
       |
       v
  steveros_hardware.cpp: read()                  [line 375]
  +-- For each joint i:
  |   fb = driver_->get_feedback(cfg.motor_id)   [line 385]
  |   position = motor_to_joint(cfg, fb.pos)     [line 387]
  |       = cfg.sign * (fb.pos - cfg.zero_offset_rad)
  |   velocity = cfg.sign * fb.velocity          [line 388]
  |   effort   = cfg.sign * fb.torque            [line 389]
       |
       v
  joint_states_[i].position  -->  ros2_control Resource Manager
                                       |
                                       v
                              JointTrajectoryController
                                       |
                                       v
                                  RViz / tf2


JOINT COMMAND TO MOTOR (Write Path):
======================================

  JointTrajectoryController command
       |
       v
  steveros_hardware.cpp: write()                 [line 423]
  +-- For each joint i:
  |   target = command_position (or hold_position during Kp ramp)
  |   motor_pos = joint_to_motor(cfg, target)    [line 440]
  |       = target / cfg.sign + cfg.zero_offset_rad
       |
       v
  robstride_protocol.hpp: encode_mit_command()   [line 134]
  +-- p_des -> float_to_uint(motor_pos, -4pi, +4pi, 16)
  +-- v_des, kp, kd encoded similarly
       |
       v
  CAN MIT Command Frame (Type 1) --> Motor
```

### 2.2 What Already Works

1. **Offset math is correct and symmetric** (`steveros_hardware.cpp:460-468`):
   - `motor_to_joint`: `sign * (motor_rad - zero_offset_rad)`
   - `joint_to_motor`: `joint_rad / sign + zero_offset_rad`
   - Round-trip: `joint_to_motor(motor_to_joint(m)) = m` (verified algebraically)

2. **Per-joint configuration** in URDF (`steveros_ros2_control.xacro`):
   - Each of 20 joints has `zero_offset_deg`, `sign`, `motor_id`, `motor_type` params
   - Offset loaded in degrees, converted to radians at init (`line 78`)

3. **Sign parameter** correctly handles direction reversal:
   - Applied to position (via `motor_to_joint`), velocity, and effort
   - Supports left/right limb symmetry

4. **Activation sequence** correctly uses offsets (`steveros_hardware.cpp:226-253`):
   - Phase 2 reads initial motor positions and applies `motor_to_joint`
   - Sets `command_position` and `hold_position` to the offset-adjusted value
   - Robot holds its current physical position on activation regardless of offset

### 2.3 What's Missing

| Gap | Impact | Location |
|-----|--------|----------|
| All 20 `zero_offset_deg` values are 0 | RViz shows wrong angles | `steveros_ros2_control.xacro` lines 24,41,58,75,92,111,128,145,162,179,198,215,232,249,266,285,302,319,336,353 |
| All 20 `sign` values are 1 | Some joints may have reversed direction | Same file, lines 23,40,57,74,91,110,127,144,161,178,197,214,231,248,265,284,301,318,335,352 |
| No calibration tool or script | Operator must manually calculate and enter offsets | N/A |
| No firmware-level zeroing support | Cannot use Robstride Type 6 SetZero command | `robstride_protocol.hpp` — Type 6 not implemented |
| Offline motors zeroed to 0.0, not offset-adjusted | Discontinuity possible when motor reconnects | `steveros_hardware.cpp:231-242` |

---

## 3. Reference Project Analysis

### 3.1 K-Scale KBot

- **URL:** https://github.com/kscalelabs/kbot
- **Relevance:** HIGHEST — same Robstride RS03/RS04 motors as SteveROS
- **Documentation:** https://docs.kscale.dev/robots/k-bot/zeroing/

**Calibration approach:** Two-level hybrid system. First, a firmware-level zero is set via CAN Type 6 command (`robstride zero`) with the robot physically positioned in its zero pose. Second, a per-joint offset lookup table (documented in their zeroing guide) accounts for the mechanical difference between motor electrical zero and desired joint zero.

**Zeroing procedure:**
1. Connect motors via USB-CAN debugging tool
2. Assign CAN IDs via Motor Studio
3. Flash latest firmware
4. Write zero offset values to each motor's parameter table
5. Position robot in defined zeroing stance (two-person operation)
6. Execute: `robstride -i can0,can1,...,can6 zero --ids 11,12,...,45`
7. Verify with state query

**Offset values from their documentation:**

| Actuator ID | Joint | Offset (rad) |
|-------------|-------|--------------|
| 11 | L Shoulder Pitch | -3.491 |
| 12 | L Shoulder Roll | -1.658 |
| 14 | L Elbow Pitch | 0 |
| 31 | L Hip Pitch | -2.216 |
| 34 | L Knee Pitch | 0 |
| 41 | R Hip Pitch | 2.216 |

**Key code (Python zeroing script):**
```python
# From kos-kbot/scripts/zero_in_place.py
await kos.actuator.configure_actuator(
    actuator_id=id,
    torque_enabled=False,
    zero_position=True  # Triggers firmware Type 6 zero write
)
```

**Applicability to SteveROS:** Directly applicable — same motors, same CAN protocol. The Type 6 CAN frame format can be implemented in SteveROS's protocol header. Their offset table provides useful reference values for joints shared between the two robots.

**Strengths:** Firmware-level persistence; simple single-command zeroing; proven on same hardware.
**Weaknesses:** Requires two people; no automatic homing; offsets hardcoded in docs rather than config file; relies entirely on motor firmware with no software-level offset fallback.

---

### 3.2 PAL Robotics TALOS

- **URL:** https://github.com/pal-robotics/talos_robot
- **Relevance:** HIGH — production 32-DOF humanoid with ros2_control

**Calibration approach:** Three-layer offset system — the most sophisticated of all projects studied.

**Layer 1 — URDF Calibration Package (`talos_description_calibration`):**
Per-robot calibration constants stored as xacro properties. Each joint gets a full 6-DOF offset (x, y, z, roll, pitch, yaw):

```xml
<xacro:property name="arm_left_1_joint_offset" value="0.0" />
<xacro:property name="head_1_joint_x_offset" value="0.0" />
<xacro:property name="head_1_joint_y_offset" value="0.0" />
```

**Layer 2 — Transmission Offsets:**
Offsets embedded in `<transmission>` definitions using ros2_control's native `<offset>` tag:

```xml
<transmission name="${name}_${side}_${number}_trans">
  <plugin>transmission_interface/SimpleTransmission</plugin>
  <joint name="${name}_${side}_${number}_joint" role="joint1">
    <offset>${offset_value}</offset>
    <mechanical_reduction>${reduction}</mechanical_reduction>
  </joint>
  <actuator name="${name}_${side}_${number}_actuator" role="actuator1"/>
</transmission>
```

**Layer 3 — Runtime Init Offset Controller:**
Proprietary `ReemcInitOffsetController` performs automatic offset calibration at startup using force-torque sensors and IMU:

```yaml
init_offset_controller:
  type: "reemc_controllers/ReemcInitOffsetController"
  joints: [leg_left_1_joint, leg_left_2_joint, ...]
  left_ft_sensor: left_ankle_ft
  right_ft_sensor: right_ankle_ft
  base_imu_sensor: base_imu
```

**Applicability to SteveROS:** The transmission offset pattern (Layer 2) is the canonical ros2_control approach and could be adopted long-term. However, for direct-drive 1:1 motors, the added complexity is not justified. The auto-calibration controller (Layer 3) requires FT sensors and IMU that SteveROS may not have.

**Strengths:** Industry gold standard; full 6-DOF offsets; automatic startup calibration; per-robot calibration packages.
**Weaknesses:** Init offset controller is proprietary; requires FT sensors/IMU; complex multi-layer system.

---

### 3.3 Pollen Robotics Reachy 2

- **URL:** https://github.com/pollen-robotics/reachy2_core
- **Relevance:** HIGH — Dynamixel + EtherCAT dual-bus ros2_control

**Calibration approach:** Per-robot YAML config file (`~/.reachy.yaml`) + hardware zeroing script for custom Orbita 3-disk actuators.

**Key zeroing utility:**
```python
# From reachy_utils/orbita_zero_hardware.py
def get_orbita_current_position(serial_port, id, reduction):
    disks = struct.unpack("fff", bytearray(resp.parameters))
    disks = [d / reduction for d in disks]
    return {"top": disks[0], "middle": disks[1], "bottom": disks[2]}

def update_config_with_zero(config_file, orbita_name, zero):
    config[f"{orbita_name}_orbita_zero"] = zero
    with open(config_file, "w") as f:
        yaml.dump(config, f, sort_keys=False)
```

**Applicability to SteveROS:** The per-robot YAML config pattern is clean and simple. A `calibration.yaml` file loaded at startup would be a good fit for SteveROS.

**Strengths:** Simple YAML config; automatic backup before writing; CLI tool with sensible defaults.
**Weaknesses:** Config is robot-local (not version-controlled); limited to Orbita actuators in open-source code.

---

### 3.4 iCub / ergoCub (IIT Robotology)

- **URL:** https://github.com/robotology/icub-models
- **Relevance:** MEDIUM — mature research humanoid, 20+ years of refinement
- **Calibration docs:** https://icub-tech-iit.github.io/documentation/icub_robot_calibration/

**Calibration approach:** Most mature system — supports 12 different calibration types for different encoder/actuator combinations. Per-robot, per-body-part XML calibration files.

**Configuration format:**
```xml
<group name="CALIBRATION">
  <param name="calibrationType">12 12 12 12 5 12</param>
  <param name="calibration1">12895 19391 57775 50843 -1500 28767</param>
  <param name="calibrationZero">0 0 0 0 0 0</param>
  <param name="calibrationDelta">0 0 0 0 0 0</param>
</group>
```

**Key concept — calibrationDelta:** Separates raw calibration from fine-tuning. `Delta = Theta_Measured - Theta_Desired`. This allows iterative refinement without touching raw calibration values.

**Calibration workflow:**
1. Set `skipCalibration=true` and `useRawEncoderData=true`
2. Place robot in zero position by hand
3. Read raw encoder values via `yarpmotorgui`
4. Enter values into `bodyPart_calib.xml`
5. Set `skipCalibration=false`, restart
6. Fine-tune with `calibrationDelta`

**Applicability to SteveROS:** The `calibrationDelta` concept (separating coarse and fine offsets) is valuable for iterative calibration. The overall workflow (position, read, calculate, enter) is exactly what SteveROS needs.

**Strengths:** Most refined system; 12 calibration types; `calibrationDelta` for fine-tuning; comprehensive documentation.
**Weaknesses:** YARP-native (not ros2_control); XML format is verbose; complex type system.

---

### 3.5 ODrive ros_odrive

- **URL:** https://github.com/odriverobotics/ros_odrive
- **Relevance:** MEDIUM — CAN-based ros2_control reference

**Calibration approach:** Explicitly does not handle calibration. From README: "It assumes that the ODrive is already configured and calibrated." All encoder calibration is done at the ODrive firmware level using `odrivetool` before the ROS2 stack starts.

**Position handling (no software offset):**
```cpp
// Direct passthrough: turns to radians, no offset
pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
```

**Applicability to SteveROS:** Demonstrates clean separation of concerns (calibration is firmware, ROS is control), but provides no software offset mechanism. Less useful as a reference for SteveROS's offset handling.

**Strengths:** Clean minimal interface; clear firmware/ROS separation.
**Weaknesses:** No software offset support; no calibration backup through ROS2.

---

### 3.6 ROBOTO ORIGIN

- **URL:** https://github.com/Roboparty/roboto_origin
- **Relevance:** MEDIUM — full-stack open-source humanoid, DM motors via CAN

**Calibration approach:** Interactive per-motor zeroing script + 3D-printed calibration jigs. Best model for SteveROS's calibration tool.

**Software offset applied in both directions:**
```cpp
// Reading positions (CAN callback):
motor_pos_ = range_map(pos_int, ...) + motor_zero_offset_;  // ADD to feedback

// Sending commands:
pos -= motor_zero_offset_;  // SUBTRACT from command
```

**Interactive zeroing tool:**
```python
# From set_zero.py
motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 0.0, 2.0, 0.0)  # Damping mode

print(">>> Please manually move motor to zero position <<<")
while True:
    motor.motor_mit_cmd(0.0, 0.0, 0.0, 2.0, 0.0)
    pos = motor.get_motor_pos()
    print(f"\rCurrent position: {pos:+.6f} rad | Press Enter to confirm...")
    if select.select([sys.stdin], [], [], 0.05)[0]:
        sys.stdin.readline()
        break

motor.set_motor_zero()  # Write zero to firmware
```

**Physical calibration jigs:** 3D-printable STEP files for each body section (shoulder, elbow, knee, ankle, waist). Ensures repeatable mechanical alignment.

**Offset storage (YAML):**
```yaml
motor_zero_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**Applicability to SteveROS:** Most directly applicable architecture. The interactive zeroing tool is an excellent model for a SteveROS calibration script. The 3D-printed jig concept is worth considering for production calibration.

**Strengths:** Software offset in both read/write paths (cleanest pattern); interactive tool with real-time position display; calibration jigs for repeatability; YAML config for version control.
**Weaknesses:** Sequential per-motor calibration is slow for 20+ motors; Chinese-language documentation.

---

## 4. ros2_control Framework Standards

### 4.1 The Canonical Way: Transmission Interface

The ros2_control framework provides first-class offset support through `SimpleTransmission`. The constructor signature:

```cpp
explicit SimpleTransmission(
  const double joint_to_actuator_reduction,
  const double joint_offset = 0.0);
```

**Mathematical formulas:**

| Direction | Position | Velocity | Effort |
|-----------|----------|----------|--------|
| Actuator → Joint | `x_j = x_a / n + x_off` | `v_j = v_a / n` | `tau_j = n * tau_a` |
| Joint → Actuator | `x_a = n * (x_j - x_off)` | `v_a = n * v_j` | `tau_a = tau_j / n` |

Where `n` is the reduction ratio and `x_off` is the joint offset.

**URDF configuration:**
```xml
<transmission name="joint1_transmission">
  <plugin>transmission_interface/SimpleTransmission</plugin>
  <joint name="joint1">
    <offset>0.5</offset>
    <mechanical_reduction>1.0</mechanical_reduction>
  </joint>
  <actuator name="joint1_motor">
    <role>actuator1</role>
  </actuator>
</transmission>
```

The framework's `component_parser.cpp` natively parses the `<offset>` tag from URDF transmission XML into the `JointInfo.offset` field in `hardware_info.hpp`.

### 4.2 Transmission Interface vs Hardware Interface

| Criterion | Transmission Layer | Hardware Interface |
|-----------|-------------------|-------------------|
| Gear reductions | Required | Inadequate |
| Direct-drive 1:1 | Overkill but correct | Pragmatically fine |
| Differential transmissions | Required | Cannot handle |
| Framework compliance | Full | Custom convention |
| Implementation complexity | More boilerplate (plugin loading) | Simpler |
| Community adoption | Underutilized (see below) | Most common in practice |

**Critical caveat:** From ros2_control Issue [#1881](https://github.com/ros-controls/ros2_control/issues/1881): "Transmissions are not loaded automatically by the resource manager. One has to adapt the hardware component to load and use them." The hardware interface must explicitly instantiate and use the transmission plugins. Example 8 from `ros2_control_demos` demonstrates this but only with reduction ratios, not offsets.

**Verdict for SteveROS:** The current hardware-interface approach is pragmatically correct for direct-drive 1:1 motors. Migration to the transmission layer is warranted only if gear reductions or differential couplings are added.

### 4.3 URDF `<calibration>` Tag

The URDF `<calibration>` tag (`<calibration rising="0.0"/>`) is technically parsed by `urdfdom` but is described as an "Unsupported Hidden Feature." It is used by `robot_calibration` (Mike Ferguson) and the Universal Robots ROS2 driver, but is not the standard approach for motor encoder offsets.

### 4.4 Available Calibration Packages

| Package | ROS2 Support | Purpose | Relevant to SteveROS? |
|---------|-------------|---------|----------------------|
| `SimpleTransmission` offset | Native | Actuator-to-joint mapping | Yes, but overkill for 1:1 |
| `robot_calibration` | Yes (ros2 branch) | Multi-sensor kinematic calibration | No — solves wrong problem |
| `ur_calibration` | Yes | UR DH parameter extraction | No — UR-specific |
| `rm_calibration_controllers` | ROS1 only | Homing via mechanical stop/GPIO | Concept applicable, code not |
| URDF `<calibration>` tag | Parsed, rarely used | Store reference positions | Not recommended |

### 4.5 Community Consensus

1. **For production robots with complex kinematics:** Use the transmission interface.
2. **For simple direct-drive robots:** Handling offsets in the hardware interface is common and accepted.
3. **For robots that lose encoder zero on power-off:** Implement a homing controller.
4. **The transmission interface is underutilized** due to the requirement that hardware components explicitly load transmission plugins.

---

## 5. Robstride Motor Firmware Capabilities

### 5.1 Complete CAN Protocol Command Types

SteveROS currently implements Types 1, 2, 3, 4, 17, 18. The full protocol:

| Type | Name | Description | SteveROS Status |
|------|------|-------------|----------------|
| 0 | ObtainID | Device ID discovery/broadcast | Defined, not used |
| 1 | Control | MIT mode motor control command | **Implemented** |
| 2 | Feedback | Motor feedback data | **Implemented** |
| 3 | Enable | Motor enable operation | **Implemented** |
| 4 | Stop | Motor stop (data[0]=0 stop, =1 stop+clear fault) | **Implemented** |
| 5 | MotorCali | Encoder calibration (factory) | Not implemented |
| 6 | **SetZero** | **Set mechanical zero** | **Not implemented** |
| 7 | SetID | Set CAN ID | Defined, not used |
| 8-10 | ParaWrite/Read/Update | Legacy parameter access | Not implemented |
| 11-14 | OTA* | Firmware update | Not implemented |
| 15-16 | CaliIng/CaliRst | Calibration status | Not implemented |
| 17 | SdoRead | SDO parameter read | **Implemented** |
| 18 | SdoWrite | SDO parameter write | **Implemented** |
| 19 | ParaStrInfo | Parameter string info | Defined, not used |
| 20 | MotorBrake | Brake control | Not implemented |
| 21 | FaultWarn | Fault/warning feedback | Defined, not used |

### 5.2 Set Zero Command (Type 6) — Frame Format

```
CAN Frame (29-bit Extended ID):
  Arbitration ID:
    bits [28:24] = 0x06    (SetZero command type)
    bits [23:8]  = host_id (typically 0x00FD)
    bits [7:0]   = motor_id (target motor)
    + CAN_EFF_FLAG

  Data (8 bytes):
    data[0] = 0x01  (set-zero trigger flag)
    data[1..7] = 0x00
```

**C++ implementation for SteveROS:**
```cpp
can_frame encode_set_zero(int motor_id) {
  can_frame frame{};
  frame.can_id =
    (6u << 24) |                                    // Type 6: SetZero
    (static_cast<uint32_t>(kHostId) << 8) |         // Host ID
    static_cast<uint32_t>(motor_id) |               // Motor ID
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = 1;  // Set-zero trigger byte
  return frame;
}
```

### 5.3 Persistence Behavior

- **Firmware zero persists across power cycles.** The Robstride motors use dual 14-bit absolute magnetic encoders. When SetZero (Type 6) is issued, the firmware stores the current encoder position as the zero reference in non-volatile flash/EEPROM. After power-on, the absolute encoder provides the true physical position, adjusted by the stored zero.

- **Parameter `0x2024 add_offset`** provides an additive offset on top of the firmware zero. Also persists across power cycles. Can be written via SDO Write (Type 18).

### 5.4 Mechanical Zero vs Electrical Zero

| Concept | CAN Command | Purpose | Persistence |
|---------|-------------|---------|-------------|
| Electrical Zero (encoder calibration) | Type 5 MotorCali | Aligns encoder with motor commutation angle | Yes (factory procedure) |
| Mechanical Zero (user zero) | Type 6 SetZero | Sets the "0 radians" reference point | Yes (user procedure) |
| Additive Offset | Type 18 SDO Write to 0x2024 | Fine-tune zero without overwriting base | Yes |

### 5.5 Key Motor Parameters (via SDO Read/Write)

| Index | Name | Type | Description |
|-------|------|------|-------------|
| 0x7005 | RunMode | uint8 | 0=Motion(MIT), 1=Position, 2=Speed, 3=Current |
| 0x7019 | MechPos | float | Current mechanical angle (read-only) |
| 0x701B | MechVel | float | Current mechanical velocity |
| 0x701C | VBus | float | Bus voltage |
| 0x2024 | add_offset | float | Zero position offset (read/write, persists) |

### 5.6 Firmware vs Software Offset Tradeoffs

| Factor | Firmware Zero (Type 6) | Software Offset (current SteveROS) |
|--------|----------------------|-----------------------------------|
| Persistence | Flash memory, survives power cycles | URDF/config file, loaded at startup |
| Reversibility | Destructive — overwrites stored zero | Non-destructive — change config and reload |
| Version control | Hidden in motor flash, no introspection | Git-tracked URDF parameters |
| Iteration speed | Requires CAN command + motor state change | Edit config, restart controller |
| Mode restrictions | Blocked in Profile Position mode | None |
| Recovery from error | Requires physical re-zeroing with Motor Studio | Revert config file |

**Recommendation:** Keep software offsets as the primary mechanism. Use firmware zeroing only as a one-time provisioning step during initial motor installation.

---

## 6. Comparative Analysis

| Approach | Projects Using It | Pros | Cons | Complexity | Persistence |
|----------|------------------|------|------|------------|-------------|
| **URDF parameter offset** (SteveROS current) | SteveROS, ROBOTO ORIGIN | Simple; version-controlled; no code changes needed; non-destructive | Must restart to apply changes; stored in URDF (mixes calibration with description) | Low | Config file (git) |
| **YAML config file** (Dynamixel pattern) | Reachy 2, ROBOTO ORIGIN | Separates calibration from URDF; per-robot customization; human-readable | Requires code to load from YAML; not automatically picked up by ros2_control | Low-Medium | Config file (local or git) |
| **Transmission interface** (ros2_control canonical) | PAL TALOS | Framework-native; handles reductions + offsets; transparent to controllers | Must explicitly load transmission plugins; overkill for 1:1 direct drive; more boilerplate | Medium-High | URDF `<transmission>` tags |
| **Motor firmware zero** (K-Scale pattern) | K-Scale KBot, ODrive | Persistent in motor flash; survives software changes; fastest runtime (no math) | Destructive; not version-controlled; no introspection; requires motor-specific tools | Low (one-time) | Motor EEPROM |
| **Calibration package** (robot_calibration) | Universal Robots | Sensor-based optimization; corrects full kinematic chain; systematic | Solves kinematic calibration, not encoder offsets; requires sensors (cameras, checkerboards) | High | URDF `<calibration>` tags |

---

## 7. Recommended Solution for SteveROS

### Phase 1: Immediate — Get It Working (No Code Changes)

**Goal:** Correctly calibrate all 20 joints using the existing infrastructure.

**What to change:** Only `steveros_description/urdf/steveros_ros2_control.xacro` — update the `zero_offset_deg` (and possibly `sign`) parameters for each joint.

**Step-by-step procedure:**

1. **Launch the robot with current (zero) offsets:**
   ```bash
   ros2 launch steveros_bringup steveros.launch.py
   ```

2. **Position the robot in its URDF zero pose:**
   - The URDF zero pose is defined by all joint angles = 0. Consult `steveros.urdf.xacro` for the physical meaning of zero for each joint.
   - For a humanoid, this typically means: standing straight, arms at sides, palms forward.
   - Use physical alignment aids (straight edges, spirit levels) for precision.

3. **Read raw motor positions:**
   ```bash
   ros2 topic echo /joint_states --once
   ```
   Since all offsets are currently 0, the reported joint positions ARE the raw motor positions (modulo sign). Record the position value for each joint.

4. **Calculate each joint's offset:**
   The formula is:
   ```
   zero_offset_deg = reported_position_in_degrees
   ```
   Since `motor_to_joint = sign * (motor_rad - zero_offset_rad)`, and we want `joint = 0` when the robot is in zero pose:
   ```
   0 = sign * (motor_reading - zero_offset_rad)
   zero_offset_rad = motor_reading
   zero_offset_deg = motor_reading * (180 / pi)
   ```

   **Worked example:**
   - Joint `dof_right_shoulder_pitch_03` reports position = 0.523 rad when robot is in zero pose
   - `zero_offset_deg = 0.523 * (180/pi) = 29.97`
   - Enter `<param name="zero_offset_deg">29.97</param>` in the URDF

5. **Determine sign for each joint:**
   - With offsets at 0, manually move each joint in the URDF-positive direction
   - If the reported position increases, `sign = 1`
   - If the reported position decreases, `sign = -1`
   - **Important:** If you change sign, you must re-measure the offset:
     ```
     zero_offset_rad = sign * motor_reading_at_zero_pose
     ```
     Wait — the formula is `joint = sign * (motor - offset)`. Setting `joint = 0`:
     ```
     offset = motor_reading_at_zero_pose    (sign cancels out)
     ```
     So the offset value is the same regardless of sign. Just measure the motor reading at zero pose and that's the offset.

6. **Enter offsets into URDF:**
   Edit `steveros_description/urdf/steveros_ros2_control.xacro`:
   ```xml
   <joint name="dof_right_shoulder_pitch_03">
     <param name="motor_id">21</param>
     <param name="motor_type">RS03</param>
     <param name="sign">1</param>
     <param name="zero_offset_deg">29.97</param>  <!-- was 0 -->
     ...
   </joint>
   ```

7. **Rebuild and verify:**
   ```bash
   cd ~/Desktop/steve/steveros
   colcon build --packages-select steveros_description
   ros2 launch steveros_bringup steveros.launch.py
   ```
   - Position robot in zero pose again
   - `ros2 topic echo /joint_states --once` should show ~0.0 for all joints
   - RViz model should match physical robot pose

**Files modified:** Only `steveros_description/urdf/steveros_ros2_control.xacro`
**Code changes:** None

---

### Phase 2: Standard — Calibration Script and Separate Config

**Goal:** Make calibration repeatable and maintainable without editing the URDF.

**2a. Create a calibration YAML file:**

Create `steveros_bringup/config/calibration.yaml`:
```yaml
# SteveROS Joint Calibration
# Generated by calibration script, do not edit manually
# Date: 2026-02-26
# Robot serial: STEVE-001

joint_offsets:
  dof_right_shoulder_pitch_03:
    zero_offset_deg: 29.97
    sign: 1
  dof_right_shoulder_roll_03:
    zero_offset_deg: -15.2
    sign: 1
  # ... all 20 joints
```

**2b. Modify hardware interface to load from YAML:**

In `steveros_hardware.cpp:on_init()`, add fallback loading from a YAML parameter:
```cpp
// After loading zero_offset_deg from URDF (line 78):
// If URDF value is 0, try loading from calibration override parameter
if (joint_configs_[i].zero_offset_rad == 0.0) {
  // Check for calibration override (loaded via launch file param)
  auto calib_key = "calibration." + joint.name + ".zero_offset_deg";
  // ... load from parameter server
}
```

**2c. Create a calibration script:**

Create `steveros_bringup/scripts/calibrate_offsets.py` — a ROS2 node that:
1. Subscribes to `/joint_states`
2. Prompts operator to position robot in zero pose
3. Records current joint positions
4. Calculates offsets
5. Writes `calibration.yaml`

Model this on ROBOTO ORIGIN's interactive zeroing script with real-time position display.

**2d. Add calibration verification node:**

A simple node that loads `calibration.yaml`, subscribes to `/joint_states`, and reports the maximum absolute joint position (should be near 0 when robot is in zero pose).

---

### Phase 3: Production — Long-Term Robustness

**Goal:** Firmware-level zeroing integration, automatic re-calibration workflow, drift detection.

**3a. Implement Robstride Type 6 SetZero command:**

Add to `robstride_protocol.hpp`:
```cpp
inline can_frame encode_set_zero(int motor_id) {
  can_frame frame{};
  frame.can_id =
    (6u << 24) |
    (static_cast<uint32_t>(kHostId) << 8) |
    static_cast<uint32_t>(motor_id) |
    CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memset(frame.data, 0, 8);
  frame.data[0] = 1;
  return frame;
}
```

Create a standalone firmware zeroing tool (NOT integrated into the ros2_control loop):
```bash
ros2 run steveros_hardware zero_motors --can can0 --ids 21,22,23,24,25
```

**3b. Calibration drift detection:**

Add to the hardware interface read cycle: periodic logging of the maximum deviation between commanded and actual positions. If deviation exceeds a threshold after the Kp ramp completes, log a warning suggesting re-calibration.

**3c. Hybrid firmware + software workflow:**

1. **Motor provisioning (one-time):** Use firmware zeroing tool to set Type 6 zero with robot in zero pose. This gets motor readings into a reasonable range.
2. **Fine calibration (per-deployment):** Use software offsets in `calibration.yaml` for fine-tuning. Values should be small (< 10 degrees) if firmware zero is set correctly.
3. **Re-calibration:** Re-run calibration script periodically or after mechanical maintenance.

---

## 8. Calibration Procedure (Step-by-Step, Operator-Friendly)

### Prerequisites
- Robot powered on, all motors enabled
- ROS2 workspace built and sourced
- `steveros_bringup` launch file available
- A straight edge or spirit level for alignment
- A second person to hold the robot (recommended)

### Procedure

#### Step 1: Launch with Zero Offsets
Ensure all `zero_offset_deg` values in `steveros_ros2_control.xacro` are set to 0. Launch:
```bash
ros2 launch steveros_bringup steveros.launch.py
```

#### Step 2: Position Robot in Zero Pose
Place the robot in its URDF-defined zero position. For SteveROS:
- **Standing straight** — legs fully extended, feet flat
- **Arms at sides** — shoulders at 0, elbows straight
- **Wrists neutral** — no rotation

Use physical aids (straight edges, flat surfaces) to ensure accuracy. A helper should support the robot.

#### Step 3: Record Raw Motor Positions
```bash
ros2 topic echo /joint_states --once | grep -A 1 "position:"
```

Record all 20 position values. Example output:
```
position:
- 0.523     # dof_right_shoulder_pitch_03
- -0.265    # dof_right_shoulder_roll_03
- 1.047     # dof_right_shoulder_yaw_02
- ...
```

#### Step 4: Determine Joint Signs
For each joint, with the robot free to move (Kp=0 or controller inactive):
1. Manually move the joint in the direction that the URDF defines as positive
2. Watch the reported position value
3. If position **increases** → `sign = 1`
4. If position **decreases** → `sign = -1`

Record the sign for each joint.

#### Step 5: Calculate Offsets
For each joint:
```
zero_offset_deg = raw_motor_position_at_zero_pose * (180 / pi)
```

**Example calculation:**
| Joint | Raw Motor (rad) | Offset (deg) | Sign |
|-------|----------------|-------------|------|
| right_shoulder_pitch | 0.523 | 29.97 | 1 |
| right_shoulder_roll | -0.265 | -15.18 | 1 |
| right_shoulder_yaw | 1.047 | 59.98 | -1 |

#### Step 6: Enter Offsets
Edit `steveros_description/urdf/steveros_ros2_control.xacro`:

```xml
<joint name="dof_right_shoulder_pitch_03">
  <param name="motor_id">21</param>
  <param name="motor_type">RS03</param>
  <param name="sign">1</param>                    <!-- from Step 4 -->
  <param name="zero_offset_deg">29.97</param>     <!-- from Step 5 -->
  ...
</joint>
```

Repeat for all 20 joints.

#### Step 7: Rebuild
```bash
cd ~/Desktop/steve/steveros
colcon build --packages-select steveros_description
source install/setup.bash
```

#### Step 8: Verify Calibration
```bash
ros2 launch steveros_bringup steveros.launch.py
```

1. Position robot in zero pose again
2. Check joint states:
   ```bash
   ros2 topic echo /joint_states --once
   ```
   All positions should be approximately 0.0 (within ~1-2 degrees / 0.02-0.04 rad)

3. Open RViz — the URDF model should match the physical robot pose

4. Move individual joints and verify:
   - Direction matches (positive direction in RViz = positive direction on robot)
   - Magnitude matches (joint limits are respected)

#### Step 9: Test Full Range
Send a simple trajectory to each joint and verify the physical motion matches RViz:
```bash
ros2 topic pub /right_arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['dof_right_shoulder_pitch_03'], \
    points: [{positions: [0.5], time_from_start: {sec: 2}}]}" --once
```

The shoulder should move approximately 28.6 degrees (0.5 rad) in the pitch-positive direction in both RViz and on the physical robot.

---

## 9. References

### Reference Robot Projects
- [K-Scale KBot](https://github.com/kscalelabs/kbot) — Robstride RS03/RS04 humanoid, K-OS runtime
- [K-Scale KBot Zeroing Documentation](https://docs.kscale.dev/robots/k-bot/zeroing/) — Step-by-step zeroing procedure
- [K-Scale Actuator Crate](https://github.com/kscalelabs/actuator) — Rust CAN driver for Robstride motors
- [PAL Robotics TALOS](https://github.com/pal-robotics/talos_robot) — Production 32-DOF humanoid
- [PAL TALOS Description Calibration](https://github.com/pal-robotics/talos_description_calibration) — Per-robot calibration constants
- [Pollen Robotics Reachy 2](https://github.com/pollen-robotics/reachy2_core) — Dynamixel + EtherCAT humanoid
- [iCub Robot Calibration Documentation](https://icub-tech-iit.github.io/documentation/icub_robot_calibration/) — Comprehensive calibration guide
- [iCub Models](https://github.com/robotology/icub-models) — URDF models with calibration data
- [ODrive ros_odrive](https://github.com/odriverobotics/ros_odrive) — CAN-based ros2_control reference
- [ROBOTO ORIGIN](https://github.com/Roboparty/roboto_origin) — Full-stack open-source humanoid with calibration jigs

### ros2_control Framework
- [ros2_control Documentation](https://control.ros.org) — Official documentation
- [SimpleTransmission Source](https://github.com/ros-controls/ros2_control/blob/master/transmission_interface/include/transmission_interface/simple_transmission.hpp) — Offset implementation
- [component_parser.cpp](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/component_parser.cpp) — URDF offset parsing
- [ros2_control Issue #1881](https://github.com/ros-controls/ros2_control/issues/1881) — Transmission loading discussion
- [ros2_control_demos Example 8](https://control.ros.org/rolling/doc/ros2_control_demos/example_8/doc/userdoc.html) — Transmission interface demo
- [robot_calibration](https://github.com/mikeferguson/robot_calibration) — Multi-sensor kinematic calibration
- [rm_calibration_controllers](https://index.ros.org/p/rm_calibration_controllers/) — Homing controller (ROS1)

### Robstride Motor Documentation
- [RobStride Product Information](https://github.com/RobStride/Product_Information) — Firmware and protocol specs
- [RobStride SampleProgram](https://github.com/RobStride/SampleProgram) — Reference CAN implementations
- [Seeed Studio Robstride Control Guide](https://wiki.seeedstudio.com/robstride_control/) — Motor control documentation
- [sirwart/robstride Python SDK](https://github.com/sirwart/robstride) — Python CAN driver reference
