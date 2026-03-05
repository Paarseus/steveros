# Quest 2 VR Teleoperation Integration Plan for SteveROS

*Generated 2026-03-04 — synthesized from unity_ros_teleoperation research, PickNik meta_quest_teleoperation research, ROS bridge analysis, and steveros codebase review.*

---

## 0. Approach Recommendation: Phased Strategy

### Three Options Compared

| Criterion | PickNik/meta_quest_teleoperation | leggedrobotics/unity_ros_teleoperation | Original quest2ros design |
|---|---|---|---|
| **Complexity** | LOW — single-purpose app | HIGH — 21-component framework | MEDIUM — custom Unity app |
| **Quest 2 support** | Explicit (Quest 1/2/3) | Via OpenXR (Quest 3 primary) | Yes (OVRPlugin) |
| **Dual-arm** | Right hand only documented | Both hands via TF | Both hands via topics |
| **Pose delivery** | `nav_msgs/Odometry` on `/right_controller_odom` | TF tree (`vr_origin→hand_left/hand_right`) + `/quest/joystick` (Joy) | Raw PoseStamped topics |
| **Dead-man switch** | Grip button = built-in clutch (hold to publish) | Joy axes 6,7 (grip_L/grip_R) OR gesture ("Closed_Fist") | Trigger float threshold |
| **Coord transform** | Needs manual conversion | Handled Unity-side (`.To<FLU>()`) | Manual in bridge node |
| **Message types** | ALL standard (Odometry, Bool, String) | Mostly standard (TF, Joy) + custom for hand tracking | Custom OVR2ROSInputs |
| **VR robot viz** | None | Full URDF import, camera feeds, point clouds | None |
| **Pre-built APK** | Yes (available) | No (must build from source) | No (doesn't exist) |
| **Bridge node size** | ~50 lines (Odometry→PoseStamped) | ~150 lines (tf2_ros + Joy sub) | ~100 lines (existed) |
| **XR plugin** | Oculus (not OpenXR) | OpenXR | OVRPlugin |
| **Maturity** | Simple, well-documented | v0.1.1, 224 commits, IROS 2024 paper | Never committed |

### Recommended Phased Approach

**Phase 1 (MVP): PickNik meta_quest_teleoperation — right arm only**
- Fastest path to working teleop: pre-built APK, standard message types, ~50 lines of bridge code
- Grip button = built-in clutch (hold to track, release to stop)
- Bridge: extract pose from Odometry → PoseStamped → MoveIt Servo
- Limitation: only right hand documented — single-arm teleop initially

**Phase 2: Add left arm support**
- Verify if PickNik app publishes left controller data (undocumented)
- If not: fork and add `/left_controller_odom` publisher (small C# change)
- OR: switch to leggedrobotics for Phase 3 directly

**Phase 3 (Full): leggedrobotics unity_ros_teleoperation — dual-arm + VR visualization**
- Both hands via TF (`hand_left`, `hand_right`)
- Per-hand clutch via `/quest/joystick` Joy axes (grip_L, grip_R)
- VR visualization of robot model, camera feeds
- No coordinate transform needed (handled Unity-side)
- Import steveros URDF for in-VR robot visualization

### Why this order?
1. PickNik gets teleop working in a day (pre-built APK, minimal code)
2. Single-arm proves the full pipeline: TCP→bridge→Servo→controller→hardware
3. Dual-arm and VR viz are additive — same MoveIt Servo configs reused
4. Switching Unity apps doesn't affect the ROS side (same quest2_teleop package, different topics/remapping)

---

## 1. Prerequisites Checklist

### Hardware
- [ ] Meta Quest 2 headset (charged, paired with Meta account)
- [ ] USB-C cable (for initial setup and APK sideloading)
- [ ] WiFi router (5 GHz preferred — both Quest 2 and ROS PC on same network)
- [ ] ROS 2 PC (the steveros development machine)
- [ ] SteveROS robot or mock hardware (for testing)

### Software — ROS 2 PC
- [ ] ROS 2 Humble (already installed)
- [ ] MoveIt 2 for Humble: `sudo apt install ros-humble-moveit` (includes moveit_servo)
- [ ] pick_ik solver: `sudo apt install ros-humble-pick-ik` (already used)
- [ ] tf2_ros: `sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs` (likely present)
- [ ] colcon build tools (already present)

### Software — Development Machine (Phase 1: not needed if using pre-built APK)
- [ ] Unity Hub (latest) — **only needed for Phase 3 or if building from source**
- [ ] Unity Editor **2022.3.12f1** — **only for leggedrobotics build**
- [ ] Android Build Support module for Unity
- [ ] `adb` command-line tool (`sudo apt install adb`)

### Software — Quest 2
- [ ] Meta Horizon app on phone (for developer mode)
- [ ] Developer mode enabled (see Step 2)
- [ ] SideQuest (optional — alternative to `adb` for APK install)

---

## 2. Quest 2 Developer Mode Setup

### 2.1 Create Meta Developer Account
1. Go to https://developer.oculus.com/ and sign in with your Meta account
2. Create a new Organization (any name — needed to unlock developer mode)
3. Accept the developer terms

### 2.2 Enable Developer Mode
1. Open the **Meta Horizon** app on your phone
2. Ensure your Quest 2 is paired (Devices → your headset)
3. Go to **Devices → [Your Quest 2] → Settings → Developer Mode**
4. Toggle **Developer Mode ON**
5. Restart the Quest 2

### 2.3 Verify ADB Connection
1. Connect Quest 2 to PC via USB-C
2. Put on headset — accept "Allow USB debugging" prompt
3. On PC:
   ```bash
   adb devices
   ```
4. Expected output:
   ```
   List of devices attached
   1WMHH8XXXXXX    device
   ```

**Verification:** Quest 2 appears in `adb devices` with status "device" (not "unauthorized").

---

## 3. Unity App Setup and APK Deployment

### Phase 1: PickNik meta_quest_teleoperation (Quickest)

#### 3A.1 Option A — Pre-built APK (fastest)
If a pre-built APK is available from the PickNik repo:
```bash
adb install meta_quest_teleoperation.apk
```

#### 3A.2 Option B — Build from Source
1. Clone the repo:
   ```bash
   git clone https://github.com/PickNikRobotics/meta_quest_teleoperation.git
   cd meta_quest_teleoperation
   ```
2. Open in Unity (version per repo docs)
3. **Build Settings → Android** → Switch Platform
4. **XR Plugin Management → Disable OpenXR, Enable Oculus**
   - Important: PickNik uses the Oculus plugin, NOT OpenXR
5. **Robotics → ROS Settings** → set ROS PC IP and port 10000
6. Connect Quest 2 via USB → **Build and Run**

#### 3A.3 PickNik Quest 2 Specifics
- If Quest 2 is not detected: in XR Plugin Management, ensure "Meta Quest 2" target is enabled
- PickNik docs note: "If you have a Quest 1, make sure to disable 'Quest 3'" — same logic applies: disable Quest 3 if only using Quest 2

#### 3A.4 Launch on Quest 2
1. App Library → **Unknown Sources** (top-right dropdown)
2. Launch the app
3. Controller mapping:
   - **Grip button** = clutch (hold to track, release to stop) — this IS the dead-man switch
   - **Trigger** = gripper toggle
   - **A button** = start demonstration
   - **B button** = stop demonstration
   - **Analog stick** = frame rotation

**Verification:** App launches. Holding grip publishes `/right_controller_odom`. Releasing grip stops publishing.

---

### Phase 3: leggedrobotics unity_ros_teleoperation (Full Featured)

#### 3B.1 Install Unity 2022.3.12f1
1. Install Unity Hub from https://unity.com/download
2. In Unity Hub → Installs → Install Editor → Archive → **2022.3.12f1**
3. During install, check **Android Build Support** (includes SDK, NDK, JDK)
4. Also install **Linux Build Support** (if building on Linux)

#### 3B.2 Clone and Open
```bash
cd ~/
git clone https://github.com/leggedrobotics/unity_ros_teleoperation.git
cd unity_ros_teleoperation
bash setup.sh   # installs git-hooks, links to Unity Hub
```
1. Unity Hub → Projects → Open → select folder
2. Wait for asset import (several minutes first time)
3. Verify packages in Package Manager:
   - `com.unity.robotics.ros-tcp-connector` (leggedrobotics fork)
   - `com.unity.robotics.visualizations`

#### 3B.3 Configure for Quest 2
1. **File → Build Settings → Android** → Switch Platform
2. **Edit → Project Settings → XR Plug-in Management**
   - Android tab → enable **OpenXR** (NOT Oculus — different from PickNik!)
   - Add **Meta Quest Feature** profile
   - Ensure **Meta Quest 2** target device is enabled (may default to Quest 3 only)
3. **OpenXR → Meta Quest Support settings → DISABLE "Force Remove Internet"**
   - CRITICAL: This setting strips internet permissions from the APK, breaking TCP connection
4. **Edit → Project Settings → Player → Android tab**
   - Minimum API Level: **API Level 29** (Android 10)
   - Target API Level: **API Level 32+**
   - Scripting Backend: **IL2CPP**
   - Target Architectures: **ARM64** only
5. Verify `Assets/Plugins/Android/AndroidManifest.xml` includes internet permissions:
   ```xml
   <uses-permission android:name="android.permission.INTERNET" />
   ```

#### 3B.4 Select Components
Enable only what's needed (reduces overhead on Quest 2 XR2 Gen 1):

**ESSENTIAL:**
- **HeadsetPublisher** — publishes TF: `hand_left`, `hand_right`, `headset`
- **TFSystem** — transform management
- **Menu** — in-VR TCP connection config (IP/port entry)

**RECOMMENDED:**
- **Hands** — hand tracking with gesture detection
- **CameraView** — stream robot camera to VR headset

**DISABLE (not needed for teleop):**
- GridMap, Voxblox, NeRFViewer, PathStreaming, AudioStreamer, ServiceCaller, Markers, VRStreamer, Haptics, Lidar, StereoImage, PoseStreaming, PosePublisher

#### 3B.5 Configure ROS Connection
1. Find **ROSConnection** object in the scene
2. Set **ROS IP** to ROS 2 PC IP (e.g., `192.168.1.100`)
3. Set **Port** to `10000`
4. This is also configurable at runtime via the in-VR Menu component

#### 3B.6 (Optional) Import SteveROS Robot Model for VR Visualization
1. Generate URDF: `ros2 run xacro xacro steveros.urdf.xacro > steveros.urdf`
2. Copy URDF + all STL meshes to `Assets/Components/Robots/Meshes/steveros/`
3. Unity → right-click URDF → Import
4. Apply URDFConverter.cs to clean simulation components
5. Register in Model Manager (Settings dropdown)
6. **Optimization:** steveros has 20 DOF + complex meshes — may need mesh decimation in Blender (target <300k vertices, <200k triangles) for Quest 2 performance

#### 3B.7 Build and Deploy
1. Connect Quest 2 via USB, approve debugging prompt
2. **File → Build Settings → Build and Run**
3. Or build APK and sideload: `adb install steveros_teleop.apk`
4. App appears under **Unknown Sources** in Quest 2 library
5. In-VR: Menu → Connection Settings → enter host IP → green dot = connected

**Verification:** App launches, VR menu shows green connection status, `ros2 topic list` shows `/tf` with `hand_left`/`hand_right` frames.

---

## 4. ROS 2 Side Setup

### 4.1 Clone ros_tcp_endpoint
```bash
cd /home/nemo/Desktop/steve/steveros
git clone -b main-ros2 https://github.com/leggedrobotics/ROS-TCP-Endpoint.git ros_tcp_endpoint
```
Note: The leggedrobotics fork is preferred — it may have patches for their Unity components. Falls back to the upstream Unity-Technologies version if needed.

### 4.2 Create quest2_teleop Package
```bash
cd /home/nemo/Desktop/steve/steveros
mkdir -p quest2_teleop/quest2_teleop
mkdir -p quest2_teleop/config
mkdir -p quest2_teleop/launch
```

Package structure:
```
quest2_teleop/
  package.xml
  setup.py
  setup.cfg
  quest2_teleop/
    __init__.py
    quest2_teleop_node.py       # Phase 1: Odometry bridge / Phase 3: TF bridge
  config/
    servo_right_arm.yaml
    servo_left_arm.yaml
  launch/
    quest2_teleop.launch.py
```

### 4.3 quest2_teleop_node.py — Bridge Node

The bridge node design differs between Phase 1 and Phase 3:

#### Phase 1 (PickNik): Odometry → PoseStamped (~50 lines)
- Subscribe to `/right_controller_odom` (`nav_msgs/Odometry`)
- The PickNik app only publishes when grip is held (built-in clutch) — no dead-man logic needed in bridge
- Extract `pose.pose` from Odometry, wrap as PoseStamped
- Apply workspace scaling (delta from origin)
- Publish to `/servo_right_arm/pose_target_cmds`
- Coordinate frame: may need Y-up→Z-up transform (PickNik does NOT use `.To<FLU>()` like leggedrobotics)

#### Phase 3 (leggedrobotics): TF + Joy → PoseStamped (~150 lines)
- Use `tf2_ros.Buffer` + `TransformListener` to look up `hand_right` and `hand_left` in robot frame
- Subscribe to `/quest/joystick` (`sensor_msgs/Joy`) — axes 6,7 are `grip_L`/`grip_R` (clutch per hand)
- Run a timer at ~50 Hz (matching Servo's 0.02s period)
- Each tick per hand:
  1. Look up transform `Torso_Side_Right → hand_right` (or `hand_left`)
  2. Check Joy grip axis > 0.5 (clutch active)
  3. If clutch released: update origin pose, skip publish
  4. If clutch active: compute workspace-scaled delta from origin
  5. Publish PoseStamped to `/servo_right_arm/pose_target_cmds` (or left)
- No coordinate transform needed (`.To<FLU>()` already applied Unity-side)
- Alternative dead-man: subscribe to `/quest/hand_gesture` and gate on "Closed_Fist" (less reliable than Joy grip)

#### VR↔Robot Frame Calibration (Both Phases)
The TF tree must connect VR frames to the robot frame. Publish a static transform:
```bash
ros2 run tf2_ros static_transform_publisher 0.5 0 0.3 0 0 0 Torso_Side_Right vr_origin
```
This places the VR origin 0.5m forward and 0.3m above the robot torso. Adjust based on where the operator physically stands relative to the robot.

For Phase 1 (PickNik), the bridge node should handle this internally — capture the first controller pose as origin, compute deltas from there.

### 4.4 MoveIt Servo Configuration

Two Servo config files, one per arm:

**config/servo_right_arm.yaml:**
```yaml
# MoveIt Servo config for right arm VR teleoperation
publish_period: 0.02          # 50 Hz output
max_expected_latency: 0.1
command_in_type: "speed_units"
scale:
  linear: 0.3                 # Reduce to 0.1 for real hardware first test
  rotational: 0.8
  joint: 0.5

move_group_name: "right_arm"
planning_frame: "Torso_Side_Right"
ee_frame_name: "KB_C_501X_Right_Bayonet_Adapter_Hard_Stop"

command_out_type: trajectory_msgs/JointTrajectory
command_out_topic: /right_arm_controller/joint_trajectory
publish_joint_positions: true
publish_joint_velocities: true
publish_joint_accelerations: false

incoming_command_timeout: 0.5   # Stop if no command for 500ms (safety)
use_smoothing: false
is_primary_planning_scene_monitor: false

# Singularity handling
lower_singularity_threshold: 10.0
hard_stop_singularity_threshold: 30.0
leaving_singularity_threshold_multiplier: 2.0

# Joint limits
joint_limit_margins: [0.1]

# Collision checking
check_collisions: true
check_octomap_collisions: false
collision_check_rate: 10.0
self_collision_proximity_threshold: 0.01
scene_collision_proximity_threshold: 0.02

# Input topics (Servo default names, namespaced per node)
cartesian_command_in_topic: ~/delta_twist_cmds
joint_command_in_topic: ~/delta_joint_cmds
joint_topic: /joint_states
status_topic: ~/status
```

**config/servo_left_arm.yaml:** — identical except:
```yaml
move_group_name: "left_arm"
ee_frame_name: "KB_C_501X_Left_Bayonet_Adapter_Hard_Stop"
command_out_topic: /left_arm_controller/joint_trajectory
```

### 4.5 Launch File

The launch file starts 4 nodes (Phase 1: 3 if single-arm):

1. **ros_tcp_endpoint** — TCP server on port 10000
2. **servo_right_arm** — MoveIt Servo instance (namespaced `/servo_right_arm`)
3. **servo_left_arm** — MoveIt Servo instance (namespaced `/servo_left_arm`) [Phase 3]
4. **quest2_teleop_node** — bridge node
5. **static_transform_publisher** — VR origin → robot frame calibration

Each Servo node requires parameters:
- `robot_description` (from xacro)
- `robot_description_semantic` (SRDF)
- `robot_description_kinematics` (kinematics.yaml)
- Servo config nested under `moveit_servo` parameter namespace

Launch arguments:
- `use_mock_hardware` (default: true)
- `host_ip` (default: 0.0.0.0) — ros_tcp_endpoint bind address
- `host_port` (default: 10000) — ros_tcp_endpoint port
- `workspace_scale` (default: 0.5) — VR→robot workspace scaling factor

### 4.6 Build
```bash
cd /home/nemo/Desktop/steve/steveros
colcon build --packages-select ros_tcp_endpoint quest2_teleop
source install/setup.bash
```

**Verification:** `colcon build` succeeds. `ros2 pkg list | grep quest2` shows `quest2_teleop`.

---

## 5. Networking Setup

### 5.1 Same WiFi Network
Both Quest 2 and ROS 2 PC must be on the **same WiFi network** (same subnet).

1. Find ROS 2 PC IP:
   ```bash
   ip addr show | grep "inet " | grep -v 127.0.0.1
   # Example: 192.168.1.100
   ```
2. Quest 2 → Settings → Wi-Fi → connect to same network

### 5.2 Firewall Configuration
```bash
# UFW (Ubuntu):
sudo ufw allow 10000/tcp

# Or iptables:
sudo iptables -A INPUT -p tcp --dport 10000 -j ACCEPT
```

### 5.3 Verify Connectivity
On ROS PC, verify endpoint is listening:
```bash
ss -tlnp | grep 10000
```

From Quest 2 app:
- PickNik: in-VR menu has IP/port entry → shows connection status
- leggedrobotics: Menu component → Connection Settings → green dot = connected

ros_tcp_endpoint logs:
```
[INFO] Connection from 192.168.1.XXX
```

### 5.4 Performance Notes
- Use **5 GHz WiFi** — 2.4 GHz adds 10-50ms+ latency and interference
- Quest 2 supports WiFi 6 (802.11ax) — use WiFi 6 router if available
- Expected round-trip latency: 5-20ms on good 5 GHz WiFi
- ros_tcp_endpoint uses persistent TCP connection (no per-message handshake overhead)
- If latency is still an issue: consider USB-C tethering via Oculus Link (wired)

**Verification:** ros_tcp_endpoint shows "Connection from [Quest IP]". `ros2 topic list` shows VR topics.

---

## 6. Launch Configurations

### 6.1 Phase 1: Right Arm Teleop with Mock Hardware
```bash
# Terminal 1: MoveIt demo with mock hardware + ros2_control
ros2 launch steveros_moveit_config demo.launch.py

# Terminal 2: Quest 2 teleop (right arm only)
ros2 launch quest2_teleop quest2_teleop.launch.py
```

### 6.2 Phase 3: Dual-Arm Teleop with Mock Hardware
```bash
# Terminal 1: MoveIt demo
ros2 launch steveros_moveit_config demo.launch.py

# Terminal 2: Quest 2 teleop (both arms)
ros2 launch quest2_teleop quest2_teleop.launch.py
```

### 6.3 Teleop with Real Hardware
```bash
# Terminal 1: Real hardware bringup
ros2 launch steveros_bringup steveros.launch.py use_mock_hardware:=false use_rviz:=true

# Terminal 2: MoveIt move_group (connects to running controllers)
ros2 launch steveros_moveit_config move_group.launch.py use_rviz:=false

# Terminal 3: Quest 2 teleop
ros2 launch quest2_teleop quest2_teleop.launch.py use_mock_hardware:=false workspace_scale:=0.2
```

### 6.4 Teleop with MuJoCo Simulation
```bash
# Terminal 1: MuJoCo sim
ros2 launch steveros_bringup steveros.launch.py use_mock_hardware:=false sim_mujoco:=true use_rviz:=true

# Terminal 2: MoveIt move_group
ros2 launch steveros_moveit_config move_group.launch.py use_rviz:=false

# Terminal 3: Quest 2 teleop
ros2 launch quest2_teleop quest2_teleop.launch.py use_mock_hardware:=false
```

### 6.5 Key Difference from MediaPipe Teleop
The existing MediaPipe pipeline (`mediapipe_ros2_py/launch/pose_teleop.launch.py`) publishes directly to JointTrajectory via joint-space angle mapping. The Quest 2 teleop goes through **MoveIt Servo**, which provides:
- Inverse kinematics (Cartesian pose → joint positions)
- Singularity avoidance (velocity scaling near singularities)
- Self-collision checking
- Joint limit enforcement
- Smooth trajectory output

Both pipelines output to the same controllers (`right_arm_controller/joint_trajectory`, `left_arm_controller/joint_trajectory`), so they are **mutually exclusive but interchangeable**. Only run one at a time.

---

## 7. Testing and Verification Procedure

**Test each step before proceeding to the next.**

### 7.1 Test ros_tcp_endpoint Alone
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```
- Launch Quest 2 app, enter ROS PC IP
- **Verify:** ros_tcp_endpoint logs show incoming connection
- **Verify:** `ros2 topic list` shows VR topics:
  - Phase 1 (PickNik): `/right_controller_odom`, `/gripper_button`, `/demonstration_indicator`
  - Phase 3 (leggedrobotics): `/tf`, `/quest/joystick`, `/quest/hand_gesture`, `/quest/pose/headset`

### 7.2 Test VR Pose Data
Phase 1:
```bash
ros2 topic echo /right_controller_odom
```
- Hold grip button → messages appear with controller pose
- Release grip → messages stop

Phase 3:
```bash
ros2 topic echo /tf --once
```
- **Verify:** transforms for `hand_right`, `hand_left`, `headset`
- **Verify:** `ros2 run tf2_tools view_frames` shows VR TF tree (`odom→vr_origin→hand_right/hand_left/headset`)

### 7.3 Test Clutch / Dead-Man Switch
Phase 1:
```bash
ros2 topic hz /right_controller_odom
# Hold grip → ~72Hz; release → 0Hz
```

Phase 3:
```bash
ros2 topic echo /quest/joystick
# Check axes[6] (grip_L) and axes[7] (grip_R): >0.5 when gripping
```

### 7.4 Test MoveIt Servo (Without VR)
```bash
# Terminal 1: MoveIt demo
ros2 launch steveros_moveit_config demo.launch.py

# Terminal 2: Start servo node (from quest2_teleop launch or manually)
# Terminal 3: Publish a test pose target
ros2 topic pub /servo_right_arm/pose_target_cmds geometry_msgs/PoseStamped \
  "{header: {frame_id: 'Torso_Side_Right'}, pose: {position: {x: 0.2, y: -0.2, z: 0.1}, orientation: {w: 1.0}}}" --once
```
- **Verify:** Robot arm moves in RViz toward target pose
- **Verify:** `ros2 topic echo /servo_right_arm/status` — no errors

### 7.5 Test Bridge Node (With VR)
```bash
ros2 launch quest2_teleop quest2_teleop.launch.py
```
- Put on Quest 2, launch app, connect to ROS
- Hold grip (clutch) and move hand
- **Verify:** `ros2 topic echo /servo_right_arm/pose_target_cmds` shows updating PoseStamped
- Release grip
- **Verify:** Messages stop (clutch disengaged)

### 7.6 Test Full Pipeline (Mock Hardware)
```bash
# Terminal 1
ros2 launch steveros_moveit_config demo.launch.py
# Terminal 2
ros2 launch quest2_teleop quest2_teleop.launch.py
```
- In RViz: observe robot arm following VR hand movements
- **Verify:** Smooth motion, no jerking
- **Verify:** Both arms track independently (Phase 3)
- **Verify:** Releasing grip freezes the arm in place

### 7.7 Test with Real Hardware (CAUTION)

**Pre-flight checklist:**
- [ ] Mock hardware test passes completely
- [ ] `workspace_scale` reduced to 0.2 (smaller movements)
- [ ] Servo `scale.linear` reduced to 0.1 (slower Cartesian speed)
- [ ] Physical e-stop accessible
- [ ] Second person ready to cut power
- [ ] CAN bus reset done (`sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000`)

```bash
# Terminal 1: Real hardware
ros2 launch steveros_bringup steveros.launch.py use_mock_hardware:=false use_rviz:=true

# Terminal 2: MoveIt move_group
ros2 launch steveros_moveit_config move_group.launch.py use_rviz:=false

# Terminal 3: Teleop (reduced speeds)
ros2 launch quest2_teleop quest2_teleop.launch.py use_mock_hardware:=false workspace_scale:=0.2
```

---

## 8. Known Issues and Workarounds

### 8.1 Quest 2 vs Quest 3
- Quest 2: Snapdragon XR2 Gen 1, grayscale passthrough, WiFi 6
- Quest 3: XR2 Gen 2, color passthrough, depth sensor, better hand tracking
- For teleop: Quest 2 is sufficient — no Quest 3 features required
- OpenXR API is shared; both Unity apps work on both headsets
- Quest 2 hand tracking: slightly lower accuracy (no depth camera assist) — use controller tracking for reliability

### 8.2 XR Plugin Mismatch Between Approaches
- **PickNik uses Oculus plugin** — disable OpenXR, enable Oculus in XR Plugin Management
- **leggedrobotics uses OpenXR** — enable OpenXR, add Meta Quest Feature profile
- Cannot use both simultaneously in the same Unity project
- This is a Unity project setting, not a device limitation

### 8.3 "Force Remove Internet" (leggedrobotics only)
- **Symptom:** Quest 2 app launches but cannot connect to ros_tcp_endpoint
- **Cause:** OpenXR Meta Quest Support plugin strips `android.permission.INTERNET` from manifest
- **Fix:** Project Settings → OpenXR → Meta Quest Support → **Disable "Force Remove Internet"**
- Also verify `AndroidManifest.xml` has `<uses-permission android:name="android.permission.INTERNET" />`

### 8.4 WiFi Latency Spikes
- **Symptom:** Jerky robot motion, momentary freezes
- **Cause:** WiFi congestion, 2.4 GHz band, distance from router
- **Fix:** 5 GHz WiFi, closer to router, reduce other WiFi traffic
- **Mitigation:** Servo's `incoming_command_timeout: 0.5` stops robot if commands stop

### 8.5 TF Tree Disconnection (Phase 3)
- **Symptom:** quest2_teleop_node cannot look up VR hand frames relative to robot
- **Cause:** No transform connecting `vr_origin` to `Torso_Side_Right`
- **Fix:** Static transform publisher in launch file:
  ```bash
  ros2 run tf2_ros static_transform_publisher 0.5 0 0.3 0 0 0 Torso_Side_Right vr_origin
  ```
  Adjust values based on operator position relative to robot.

### 8.6 MoveIt Servo Singularity Stops
- **Symptom:** Arm freezes, Servo status reports singularity
- **Cause:** 5-DOF arms have more singular configurations than 6/7-DOF
- **Fix:** Increase `hard_stop_singularity_threshold` from 30 to 50 (more permissive)
- **Help:** pick_ik with `rotation_scale: 0.0` (position-only IK) reduces singularity sensitivity

### 8.7 pick_ik Timeout vs Servo Rate
- **Symptom:** Sluggish tracking of fast hand movements
- **Cause:** IK solve timeout (50ms) + Servo period (20ms) = ~70ms per step
- **Fix:** Reduce `kinematics_solver_timeout` to 0.02 (20ms), set `kinematics_solver_attempts: 1`

### 8.8 Controller Conflicts
- **Symptom:** Two teleop sources fight over arm controllers
- **Fix:** Only run ONE source at a time — kill `pose_to_joints` (MediaPipe) before starting Quest 2 teleop

### 8.9 Quest 2 Guardian System
- **Symptom:** App pauses when moving outside Guardian boundary
- **Fix:** Set up room-scale Guardian boundary. Teleop works seated or standing.

### 8.10 ros_tcp_endpoint Connection Refused
- **Checklist:**
  1. Same WiFi network? (`ping` ROS PC IP from phone on same network)
  2. Firewall open? (`sudo ufw allow 10000/tcp`)
  3. ros_tcp_endpoint running? (`ros2 node list | grep tcp`)
  4. Correct IP in Unity app settings?
  5. Not using VPN/Docker network isolation?
  6. (leggedrobotics) "Force Remove Internet" disabled? (see 8.3)

### 8.11 APK Install Issues
- **"INSTALL_FAILED_USER_RESTRICTED":** Quest 2 → Settings → Developer → "Install unknown apps" → allow
- **APK not visible:** App Library → **Unknown Sources** (dropdown in top-right)
- **SideQuest shows green dot** = connected (use for drag-and-drop APK install)

### 8.12 Quest 2 Performance with leggedrobotics VR Robot Model
- **Symptom:** Low FPS, stuttering in VR
- **Cause:** steveros mesh complexity too high for Quest 2's XR2 Gen 1
- **Fix:** Decimate meshes in Blender (target <300k vertices, <200k triangles per model). Disable unnecessary components. Use LOD (Level of Detail) if available.

### 8.13 Coordinate Transform (PickNik only)
- **Symptom:** Robot arm moves in wrong direction relative to hand movement
- **Cause:** PickNik app does NOT apply `.To<FLU>()` — Unity Y-up left-handed coords arrive raw
- **Fix:** Apply transform in bridge node: `ros_x = unity_z`, `ros_y = -unity_x`, `ros_z = unity_y` (same as the original quest2_teleop_node's `_transform_quest_to_ros` function)

---

## 9. Architecture Diagrams

### Phase 1: PickNik — Single Arm MVP

```
┌─────────────────────────────────────────────────────────┐
│  META QUEST 2                                           │
│  ┌───────────────────────────────────────┐              │
│  │  PickNik meta_quest_teleoperation     │              │
│  │  ├─ /right_controller_odom (Odometry) │              │
│  │  ├─ /gripper_button (Bool)            │              │
│  │  └─ Grip held = publish (clutch)      │              │
│  └──────────────┬────────────────────────┘              │
│                 │ TCP :10000                              │
└─────────────────┼────────────────────────────────────────┘
                  │ WiFi (5 GHz)
┌─────────────────┼────────────────────────────────────────┐
│  ROS 2 PC       │                                        │
│  ┌──────────────▼─────────────┐                          │
│  │  ros_tcp_endpoint          │                          │
│  └──────────────┬─────────────┘                          │
│                 │ /right_controller_odom                   │
│  ┌──────────────▼─────────────┐                          │
│  │  quest2_teleop_node        │                          │
│  │  ├─ Odometry → PoseStamped │                          │
│  │  ├─ Y-up → Z-up transform  │                          │
│  │  └─ Workspace scaling       │                          │
│  └──────────────┬─────────────┘                          │
│  ┌──────────────▼─────────────┐                          │
│  │  servo_right_arm           │  MoveIt Servo            │
│  │  PoseStamped → JointTraj   │  IK + collision check    │
│  └──────────────┬─────────────┘                          │
│  ┌──────────────▼─────────────┐                          │
│  │  right_arm_controller      │  JointTrajectoryCtrl     │
│  └──────────────┬─────────────┘  (100 Hz)                │
│  ┌──────────────▼─────────────┐                          │
│  │  steveros_hardware         │  CAN / mock / MuJoCo     │
│  └────────────────────────────┘                          │
└──────────────────────────────────────────────────────────┘
```

### Phase 3: leggedrobotics — Dual-Arm Full Featured

```
┌──────────────────────────────────────────────────────────────────┐
│  META QUEST 2                                                    │
│  ┌──────────────────────────────────────────────┐                │
│  │  unity_ros_teleoperation                     │                │
│  │  ├─ HeadsetPublisher → /tf (hand_L/R, head) │                │
│  │  ├─ /quest/joystick (Joy: grip_L, grip_R)   │                │
│  │  └─ .To<FLU>() applied (ROS coords)         │                │
│  └──────────────┬───────────────────────────────┘                │
│                 │ TCP :10000                                      │
└─────────────────┼────────────────────────────────────────────────┘
                  │ WiFi (5 GHz)
┌─────────────────┼────────────────────────────────────────────────┐
│  ROS 2 PC       │                                                │
│  ┌──────────────▼───────────────┐                                │
│  │  ros_tcp_endpoint            │                                │
│  └──────────────┬───────────────┘                                │
│        /tf + /quest/joystick                                      │
│  ┌──────────────▼───────────────┐                                │
│  │  quest2_teleop_node          │                                │
│  │  ├─ tf2_ros: hand_R/L poses  │                                │
│  │  ├─ Joy grip axes (clutch)   │                                │
│  │  └─ Workspace scaling        │                                │
│  └──────┬───────────────┬───────┘                                │
│  ┌──────▼──────┐ ┌──────▼──────┐                                 │
│  │servo_right  │ │servo_left   │  MoveIt Servo (2 instances)     │
│  │→ JointTraj  │ │→ JointTraj  │  IK + collision + singularity   │
│  └──────┬──────┘ └──────┬──────┘                                 │
│  ┌──────▼──────┐ ┌──────▼──────┐                                 │
│  │right_arm_   │ │left_arm_    │  JointTrajectoryController      │
│  │controller   │ │controller   │  (ros2_control, 100 Hz)         │
│  └──────┬──────┘ └──────┬──────┘                                 │
│  ┌──────▼───────────────▼──────┐                                 │
│  │  steveros_hardware          │  CAN bus / mock / MuJoCo        │
│  └─────────────────────────────┘                                 │
└──────────────────────────────────────────────────────────────────┘
```

---

## 10. Implementation Order

### Phase 1: Right Arm MVP (target: 1-2 days)

| Step | Task | Test |
|------|------|------|
| 1 | Enable Quest 2 developer mode | `adb devices` shows device |
| 2 | Install PickNik APK (pre-built or build) | App launches on Quest 2 |
| 3 | Clone ros_tcp_endpoint into workspace | `colcon build` succeeds |
| 4 | Create quest2_teleop package scaffold | `colcon build` succeeds |
| 5 | Write servo_right_arm.yaml | File parses without errors |
| 6 | Write quest2_teleop_node.py (Odometry→PoseStamped bridge) | Node starts, subscribes |
| 7 | Write quest2_teleop.launch.py (3 nodes) | Launch starts all nodes |
| 8 | Open firewall, test WiFi connectivity | App connects to ros_tcp_endpoint |
| 9 | Test with mock hardware in RViz | Right arm tracks VR hand in RViz |
| 10 | Tune workspace_scale, Servo speeds | Motion feels responsive |
| 11 | Test with MuJoCo simulation | Arm moves in MuJoCo |
| 12 | Test with real hardware (reduced speeds, e-stop ready) | Physical arm tracks VR hand |

### Phase 2: Add Left Arm (target: 1 day)

| Step | Task | Test |
|------|------|------|
| 13 | Write servo_left_arm.yaml | File parses |
| 14 | Verify/add left controller topic from PickNik app | `/left_controller_odom` publishes |
| 15 | Update bridge node for dual-arm | Both arms track independently |
| 16 | Update launch file (add left Servo node) | 4 nodes launch |
| 17 | Test dual-arm with mock hardware | Both arms follow VR hands |

### Phase 3: Full VR Framework (target: 3-5 days)

| Step | Task | Test |
|------|------|------|
| 18 | Install Unity 2022.3.12f1 + Android support | Unity opens |
| 19 | Clone + configure unity_ros_teleoperation | Project opens, Android platform set |
| 20 | Configure Quest 2 XR settings + internet perms | Build succeeds |
| 21 | Select components (HeadsetPublisher, Hands, Menu) | APK launches on Quest 2 |
| 22 | Update quest2_teleop_node.py (TF + Joy bridge) | Node reads hand TF frames |
| 23 | Test dual-arm with TF-based tracking | Both arms track via TF |
| 24 | (Optional) Import steveros URDF into Unity | Robot model visible in VR |
| 25 | (Optional) Configure CameraView for robot camera | Camera feed in VR headset |

---

## 11. Files to Create/Modify

### New files (all phases):
```
quest2_teleop/
  package.xml
  setup.py
  setup.cfg
  quest2_teleop/
    __init__.py
    quest2_teleop_node.py
  config/
    servo_right_arm.yaml
    servo_left_arm.yaml         # Phase 2+
  launch/
    quest2_teleop.launch.py
```

### Cloned packages:
- `ros_tcp_endpoint/` (from leggedrobotics/ROS-TCP-Endpoint, `main-ros2` branch)

### No modifications to existing files:
- `steveros_moveit_config/` — SRDF, kinematics.yaml, joint_limits.yaml, controllers all reused as-is
- `steveros_bringup/` — hardware launch unchanged
- `steveros_description/` — URDF/xacro unchanged

### Phase 3 additional (Unity side, not in steveros repo):
- `unity_ros_teleoperation/` — cloned separately, not part of ROS workspace
- steveros URDF + meshes copied into Unity project (optional, for VR robot model)

---

## 12. Topic Reference

### Phase 1 (PickNik) — Topics on ROS 2 side

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/right_controller_odom` | `nav_msgs/Odometry` | Quest→ROS | Right controller pose (only when grip held) |
| `/gripper_button` | `std_msgs/Bool` | Quest→ROS | Trigger press toggle |
| `/demonstration_indicator` | `std_msgs/String` | Quest→ROS | "Starting/Stopping demonstration" |
| `/servo_right_arm/pose_target_cmds` | `geometry_msgs/PoseStamped` | Bridge→Servo | Converted pose target |
| `/right_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Servo→Controller | Joint commands |

### Phase 3 (leggedrobotics) — Topics on ROS 2 side

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | Quest→ROS | hand_left, hand_right, headset frames |
| `/quest/joystick` | `sensor_msgs/Joy` | Quest→ROS | 8 axes incl. grip_L[6], grip_R[7] |
| `/quest/hand_gesture` | `HandGestureMsg` (custom) | Quest→ROS | "Closed_Fist", "Thumb_Up", etc. |
| `/quest/pose/headset` | `geometry_msgs/PoseStamped` | Quest→ROS | Headset pose |
| `/servo_right_arm/pose_target_cmds` | `geometry_msgs/PoseStamped` | Bridge→Servo | Right arm target |
| `/servo_left_arm/pose_target_cmds` | `geometry_msgs/PoseStamped` | Bridge→Servo | Left arm target |
| `/right_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Servo→Controller | Right arm joints |
| `/left_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Servo→Controller | Left arm joints |
