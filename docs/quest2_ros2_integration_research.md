# Quest 2 + ROS2 Integration Research for SteveROS

**Date:** 2026-03-05
**Workspace:** SteveROS (KBot 20-DOF Humanoid)
**Target ROS2 Distro:** Jazzy (primary), Humble (MediaPipe nodes)
**Goal:** Connect Meta Quest 2 to this ROS2 workspace for VR-based teleoperation

---

## Workspace Context

SteveROS is a full-stack ROS2 framework for a KBot 20-DOF humanoid robot featuring:
- `ros2_control` hardware interface (Robstride RS03 motors over CAN bus)
- MuJoCo simulation via `mujoco_ros2_control`
- MoveIt2 for arm motion planning (with `pick_ik`)
- MediaPipe-based vision nodes (hand tracking, pose estimation, gesture recognition)
- Default CycloneDDS middleware

The workspace already has MediaPipe hand tracking for camera-based teleoperation.
Adding Quest 2 VR would provide immersive 6-DoF controller/hand tracking with
significantly better spatial accuracy and an immersive visualization layer.

---

## Top Repositories & Projects Evaluated

### 1. unity_ros_teleoperation (ETH Zurich / Legged Robotics) -- RECOMMENDED

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/leggedrobotics/unity_ros_teleoperation |
| **License** | BSD-3-Clause |
| **ROS2 Support** | Yes (use `main-ros2` branch of ROS-TCP-Endpoint) |
| **Quest 2 Compatible** | Yes (OpenXR-based, works with Quest 2/3/Pro) |
| **Unity Version** | 2022.3.12f1 |
| **Maintained** | Active (v0.1.1, 2024) |
| **Ease of Setup** | Medium |

**Why it's the best fit for SteveROS:**
- **28+ components** including hand tracking, pose publishing, LiDAR visualization, camera streaming, elevation maps, and bidirectional audio
- **OpenXR foundation** means it works with Quest 2 (not locked to Quest 3)
- **Supports multiple robot models** out of the box (Anymal, Franka Panda, etc.) -- easy to add KBot URDF
- **ROS2 native** via TCP Endpoint on the `main-ros2` branch
- **Hand tracking** with Ability model compatibility -- can complement existing MediaPipe setup
- **GPU-accelerated** point cloud and LiDAR rendering in VR
- Published at IROS 2024 (Wilder-Smith, Patil & Hutter - "Radiance Fields for Robotic Teleoperation")

**Setup overview:**
1. Clone repo and open in Unity 2022.3.12f1
2. Build for Android (Quest 2 target)
3. On ROS2 side: clone `ROS-TCP-Endpoint` (`main-ros2` branch) into colcon workspace
4. Run: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ip>`
5. Connect Quest to same network, configure endpoint IP in-app

**Compatibility with SteveROS:**
- Works with ROS2 Jazzy (TCP Endpoint is distro-agnostic)
- Can visualize MuJoCo sim state in VR
- Can feed controller poses to MoveIt2 servo for real-time arm control
- Hand tracking data can supplement/replace MediaPipe pipeline

---

### 2. PickNikRobotics/meta_quest_teleoperation -- STRONG RUNNER-UP

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/PickNikRobotics/meta_quest_teleoperation |
| **Stars** | 7 |
| **ROS2 Support** | Yes (designed for ROS2) |
| **Quest 2 Compatible** | Yes (Meta Quest headsets) |
| **Maintained** | Active (5 commits, PickNik team) |
| **Ease of Setup** | Easy-Medium |
| **Docs** | https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/ |

**Why it's great for SteveROS:**
- **Built specifically for MoveIt** -- SteveROS already uses MoveIt2
- Publishes to `/right_controller_odom` and `/tf` -- standard ROS2 topics
- **Gripper control** via trigger, **clutch mechanism** via grip button
- **Frame rotation** via analog stick for aligning VR frame to robot frame
- Works as a generic ROS2 driver (MoveIt Pro recommended but not required)
- Open source Unity project

**Setup overview:**
1. Clone repo, open in Unity Hub (auto-fetches correct Unity version)
2. Enable Android build target, configure XR Plugin Management
3. Build and deploy to Quest 2
4. On ROS2 side: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ip>`
5. Enter PC IP in Quest app keyboard

**Published ROS2 Topics:**
- `/right_controller_odom` (nav_msgs/Odometry)
- `/tf` (geometry_msgs/TransformStamped)
- Gripper state commands

**Compatibility with SteveROS:**
- Direct integration with MoveIt2 servo for real-time arm control
- Works with ROS2 Jazzy via ROS-TCP-Endpoint
- Simple topic mapping to existing joint trajectory controllers

---

### 3. Hand Tracking Streamer (HTS) -- BEST FOR CONTROLLER-FREE TRACKING

| Field | Details |
|-------|---------|
| **Quest App** | https://www.meta.com/experiences/hand-tracking-streamer/26303946202523164/ |
| **SideQuest** | https://sidequestvr.com/app/46236/hand-tracking-streamer |
| **ROS2 Package** | `hand_tracking_sdk_ros2` (in development) |
| **Python SDK** | `pip install hand-tracking-sdk` |
| **Quest 2 Compatible** | Yes |
| **Price** | Free |
| **Posted** | February 2026 (very recent) |

**Key features:**
- **21 hand landmarks per hand** (MediaPipe-compatible format!)
- **6-DoF wrist pose** tracking
- **No controllers needed** -- bare hand tracking
- UDP (low latency) or TCP (reliable) streaming modes
- Left, right, or dual-hand modes
- In-headset live debug console and phantom hand visualization
- Built on Meta Interaction SDK

**Why it's interesting for SteveROS:**
- MediaPipe-compatible landmark format matches existing `mediapipe_ros2_py` nodes
- Could be a drop-in replacement for camera-based MediaPipe with much better accuracy
- ROS2 bridge node publishes wrist pose topics, landmark topics, and TF frames
- No Unity development required -- just install the app

**Caveats:**
- ROS2 package (`hand_tracking_sdk_ros2`) is still in active development
- Interface may change as specs finalize
- No established version compatibility matrix yet

---

### 4. Quest2ROS (KTH Royal Institute of Technology) -- ROS1 ONLY

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/Quest2ROS/quest2ros |
| **Website** | https://quest2ros.github.io/ |
| **Stars** | 36 |
| **Last Commit** | December 2023 |
| **ROS2 Support** | NO (ROS1 only, uses catkin) |
| **Quest 2 Compatible** | Yes |
| **Paper** | HRI 2024 Workshop |

**Performance metrics (published):**
- Tracking accuracy: **0.46 mm** mean error at ~1m distance
- Latency: **82 ms** average
- Update frequency: **71.86 Hz**

**NOT recommended for SteveROS** because:
- ROS1 only (`catkin build`, `rosrun`, `roslaunch`)
- Would require `ros1_bridge` which adds complexity and latency
- Not actively maintained (last commit Dec 2023)

---

### 5. vr-hand-tracking (Northwestern ME 495) -- QUEST 3 FOCUSED

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/NU-MECH-ENG-495/vr-hand-tracking |
| **Stars** | 26 |
| **ROS2 Support** | Yes |
| **Quest 2 Compatible** | Targets Quest 3/3s but may work with Quest 2 |
| **Unity Version** | Unity 6 |

**Features:**
- Captures hand joint angles via Meta OpenXR SDK
- Broadcasts via UDP to a C++ ROS2 node
- Provides RViz visualization
- Optional robotic hand remote control integration

**Caveats:**
- Primarily targets Quest 3s (newer hardware)
- Course project (may not be actively maintained long-term)
- Uses Unity 6 (newer than most other projects)

---

### 6. vr_ros2_bridge (UM ARM Lab) -- HTC VIVE FOCUSED

| Field | Details |
|-------|---------|
| **Repo** | https://github.com/UM-ARM-Lab/vr_ros2_bridge |
| **Stars** | 11 |
| **ROS2 Support** | Yes |
| **Quest 2 Compatible** | No (HTC Vive only, requires SteamVR) |

**Not recommended** -- designed for HTC Vive with SteamVR on Windows. Quest 2 uses a different ecosystem.

---

## Architecture Comparison

### Approach A: Unity + ROS-TCP-Connector (Recommended)

```
Quest 2 (Unity App) --[TCP]--> ROS-TCP-Endpoint --[ROS2 Topics]--> SteveROS
```

**Pros:**
- Most mature approach, used by ETH, PickNik, and Unity Technologies
- Bidirectional communication (can send haptic feedback back)
- Rich visualization (render robot model, sensors in VR)
- Works across ROS2 distros (TCP is transport-layer agnostic)

**Cons:**
- Requires Unity development for customization
- Must build and deploy Android APK to Quest
- ROS-TCP-Connector has known connectivity issues on Quest 2 (GitHub Issues #353, #359, #280) -- workaround: ensure same subnet, disable firewall, use static IPs

### Approach B: Hand Tracking Streamer + Python SDK (Simplest)

```
Quest 2 (HTS App) --[UDP/TCP]--> Python SDK --[ROS2 bridge]--> SteveROS
```

**Pros:**
- No Unity development at all
- Free app from Meta Store
- MediaPipe-compatible output (matches existing SteveROS pipeline)
- Lowest barrier to entry

**Cons:**
- ROS2 package still in development
- No immersive visualization (Quest only used as input device)
- No controller button input (hand tracking only)

### Approach C: WebXR + rosbridge_suite (Alternative)

```
Quest 2 (Browser/WebXR) --[WebSocket]--> rosbridge_server --[ROS2 Topics]--> SteveROS
```

**Pros:**
- No app installation on Quest
- Web-based, easy to iterate
- rosbridge_suite is well-maintained

**Cons:**
- Higher latency (WebSocket overhead)
- Limited hand tracking quality in browser
- More complex web development required
- WebXR API limitations on Quest browser

### Approach D: MQTT Bridge (Fallback)

```
Quest 2 (Unity App) --[MQTT]--> Mosquitto Broker --[MQTT-ROS2 Bridge]--> SteveROS
```

**Pros:**
- Avoids ROS-TCP-Connector connectivity issues on Quest 2
- MQTT is lightweight and reliable

**Cons:**
- Additional infrastructure (MQTT broker)
- Less common pattern, fewer examples
- Still requires Unity development

---

## Final Recommendation for SteveROS

### Primary Path: unity_ros_teleoperation (ETH Zurich)

**Why:**
1. Most feature-complete VR teleoperation framework available
2. OpenXR-based = Quest 2 compatible
3. ROS2 support via TCP Endpoint (works with Jazzy)
4. Already supports multiple robot models -- add KBot URDF
5. Hand tracking built in -- can replace or supplement MediaPipe pipeline
6. Published research backing (IROS 2024)
7. Active development by reputable robotics lab (ETH Zurich RSL)

### Quick-Start Alternative: PickNikRobotics/meta_quest_teleoperation

**Why:**
1. Simpler to set up (fewer components)
2. Designed specifically for MoveIt teleoperation (SteveROS uses MoveIt2)
3. Publishes standard odometry + TF topics
4. Can be up and running faster than the ETH solution
5. PickNik is the team behind MoveIt -- guaranteed compatibility

### Future Watch: Hand Tracking Streamer

**Why:**
1. Once the ROS2 package stabilizes, this becomes the easiest option
2. MediaPipe-compatible output = near drop-in for existing SteveROS pipeline
3. No Unity development needed
4. Free and available on Quest Store today
5. Best option if you want bare-hand tracking without controllers

---

## Integration Roadmap for SteveROS

### Phase 1: Quick Prototype (1-2 days)
1. Clone `PickNikRobotics/meta_quest_teleoperation`
2. Build and deploy to Quest 2
3. Add `ROS-TCP-Endpoint` (`main-ros2` branch) to SteveROS workspace
4. Map `/right_controller_odom` to MoveIt2 servo input
5. Test with MuJoCo sim first

### Phase 2: Full Integration (1-2 weeks)
1. Set up `unity_ros_teleoperation` (ETH)
2. Import KBot URDF into Unity project
3. Configure MuJoCo camera feeds for VR passthrough
4. Set up bidirectional communication (haptic feedback from contact forces)
5. Add hand tracking as alternative to controllers

### Phase 3: Production Pipeline
1. Evaluate Hand Tracking Streamer once ROS2 package is stable
2. Compare tracking quality: Quest hand tracking vs. MediaPipe camera
3. Integrate best option into `steveros_bringup` launch files
4. Add VR teleop mode to roadmap

---

## Key Links

| Resource | URL |
|----------|-----|
| unity_ros_teleoperation | https://github.com/leggedrobotics/unity_ros_teleoperation |
| meta_quest_teleoperation | https://github.com/PickNikRobotics/meta_quest_teleoperation |
| Hand Tracking Streamer | https://www.meta.com/experiences/hand-tracking-streamer/26303946202523164/ |
| Quest2ROS (ROS1 only) | https://github.com/Quest2ROS/quest2ros |
| vr-hand-tracking | https://github.com/NU-MECH-ENG-495/vr-hand-tracking |
| vr_ros2_bridge | https://github.com/UM-ARM-Lab/vr_ros2_bridge |
| ROS-TCP-Endpoint (ROS2) | https://github.com/Unity-Technologies/ROS-TCP-Endpoint (main-ros2 branch) |
| ROS-TCP-Connector | https://github.com/Unity-Technologies/ROS-TCP-Connector |
| MoveIt Pro Quest Guide | https://docs.picknik.ai/hardware_guides/setting_up_the_meta_quest_for_teleop/ |
| Quest2ROS Paper | https://openreview.net/forum?id=d3MTESE2e8 |
| HTS Discourse Post | https://discourse.openrobotics.org/t/hand-tracking-streamer-quest-hand-wrist-telemetry-streaming-tool/52511 |
