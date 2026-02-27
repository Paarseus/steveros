# Motor Setup & Calibration Guide

## Prerequisites

- CAN interface (gs_usb adapter) connected
- ROS2 Jazzy workspace built: `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install`

### Reset CAN Before Any Operation

Always reset CAN before starting:

```bash
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000
```

---

## Part 1: Setting Motor CAN IDs

New Robstride motors ship with a default CAN ID (typically **127**). Each motor must be assigned a unique ID before use.

### Motor ID Assignments

| Limb      | Joint              | Motor ID | Motor Type |
|-----------|--------------------|----------|------------|
| Right Arm | shoulder_pitch     | 21       | RS03       |
| Right Arm | shoulder_roll      | 22       | RS03       |
| Right Arm | shoulder_yaw       | 23       | RS02       |
| Right Arm | elbow              | 24       | RS02       |
| Right Arm | wrist              | 25       | RS02       |
| Left Arm  | shoulder_pitch     | 11       | RS03       |
| Left Arm  | shoulder_roll      | 12       | RS03       |
| Left Arm  | shoulder_yaw       | 13       | RS02       |
| Left Arm  | elbow              | 14       | RS02       |
| Left Arm  | wrist              | 15       | RS02       |
| Right Leg | hip_pitch          | 41       | RS04       |
| Right Leg | hip_roll           | 42       | RS03       |
| Right Leg | hip_yaw            | 43       | RS03       |
| Right Leg | knee               | 44       | RS04       |
| Right Leg | ankle              | 45       | RS02       |
| Left Leg  | hip_pitch          | 31       | RS04       |
| Left Leg  | hip_roll           | 32       | RS03       |
| Left Leg  | hip_yaw            | 33       | RS03       |
| Left Leg  | knee               | 34       | RS04       |
| Left Leg  | ankle              | 35       | RS02       |

### Step 1: Connect One Motor at a Time

Only connect **one new (unassigned) motor** to the CAN bus at a time to avoid ID conflicts.

### Step 2: Scan for the Motor

Send a clear-fault (Type 4, data[0]=1) to common default IDs and listen for Type 2 (feedback) responses:

```bash
candump can0 -t a > /tmp/can_scan.txt &
CPID=$!
sleep 0.2

# Scan default IDs (0, 1-5, 127)
for mid in 0 1 2 3 4 5 127; do
  cansend can0 0400FD$(printf "%02X" $mid)#0100000000000000
  sleep 0.05
done

sleep 1
kill $CPID 2>/dev/null

# Show only feedback responses (filter out sent frames)
grep -v "0400FD" /tmp/can_scan.txt
```

A response like `02007FFD` means the motor is at ID 127 (0x7F). The motor ID is in bits 8-15 of the arbitration ID.

### Step 3: Set the New CAN ID

Use the Robstride **Type 7 (SetID)** command:

**Arbitration ID format:**
```
(7 << 24) | (new_id << 16) | (host_id << 8) | current_id
```

Where `host_id` = 0xFD (253).

**Data:** 8 zero bytes.

**Example** — change motor from ID 127 to ID 11:

```bash
# arb = (7 << 24) | (0x0B << 16) | (0xFD << 8) | 0x7F = 0x070BFD7F
cansend can0 070BFD7F#0000000000000000
```

**Quick reference for computing the arb ID:**

```bash
# Variables
CURRENT_ID=127
NEW_ID=11
HOST_ID=253  # 0xFD

# Compute and send
ARB=$(printf "%02X%02X%02X%02X" 7 $NEW_ID $HOST_ID $CURRENT_ID)
cansend can0 ${ARB}#0000000000000000
```

### Step 4: Verify the New ID

```bash
candump can0 -t a > /tmp/can_verify.txt &
CPID=$!
sleep 0.2

# Send clear-fault to the new ID (e.g., 11 = 0x0B)
cansend can0 0400FD0B#0100000000000000

sleep 1
kill $CPID 2>/dev/null
grep -v "0400FD" /tmp/can_verify.txt
```

A response like `02000BFD` confirms the motor is now at ID 11.

### Step 5: Repeat for Each Motor

Disconnect the configured motor, connect the next one, and repeat from Step 2.

---

## Part 2: Zero-Offset Calibration

After all motor IDs are set, calibrate the zero offsets so that the URDF zero pose matches the physical zero pose.

### Step 1: Zero Out Current Offsets

In `steveros_description/urdf/steveros_ros2_control.xacro`, set `zero_offset_deg` to `0` for all joints being calibrated:

```xml
<param name="zero_offset_deg">0</param>
```

### Step 2: Build and Launch

```bash
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000
source install/setup.bash
ros2 launch steveros_bringup steveros.launch.py use_mock_hardware:=false use_rviz:=false
```

### Step 3: Position the Robot

Physically move the limb into its **URDF zero pose**:
- **Arms**: straight down at the sides, elbow fully extended
- **Legs**: straight, standing upright

### Step 4: Read Raw Positions

In a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic echo /joint_states --once
```

Note the `position` values (in radians) for the joints being calibrated.

### Step 5: Convert and Update Offsets

Convert each reading from radians to degrees and set as `zero_offset_deg`:

```
degrees = radians * 180 / pi
```

For example, if `dof_left_shoulder_pitch_03` reads `1.9158 rad`:

```
1.9158 * 180 / 3.14159 = 109.75 deg
```

Update in `steveros_ros2_control.xacro`:

```xml
<param name="zero_offset_deg">109.75</param>
```

### Step 6: Verify

Kill the running launch, rebuild, relaunch, and read `/joint_states` again. All calibrated joints should read approximately **0.0 rad** (within ~0.005 rad / ~0.3 deg).

---

## Useful Commands

### Stop All Motors

```bash
for mid in 21 22 23 24 25 11 12 13 14 15 41 42 43 44 45 31 32 33 34 35; do
  cansend can0 0400FD$(printf "%02X" $mid)#0000000000000000
  sleep 0.02
done
```

### Scan All Known Motor IDs

```bash
candump can0 -t a > /tmp/can_scan.txt &
CPID=$!
sleep 0.2

for mid in 11 12 13 14 15 21 22 23 24 25 31 32 33 34 35 41 42 43 44 45; do
  cansend can0 0400FD$(printf "%02X" $mid)#0100000000000000
  sleep 0.05
done

sleep 1
kill $CPID 2>/dev/null
grep -v "0400FD" /tmp/can_scan.txt
```

### Kill All ROS2 Processes

```bash
pkill -9 -f "ros2_control_node|robot_state_publisher|rviz2|rqt|spawner"
```

**Important:** Always send motor stop commands after killing ROS2 processes, since `pkill -9` doesn't trigger the deactivation handler.
