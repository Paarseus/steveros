# SteveROS Development Roadmap

*Generated 2026-02-27 from research across K-Scale ecosystem, ROS2 humanoid projects, sim-to-real pipelines, and practical robotics practice.*

## Executive Summary

The project has passed the critical "hardware interface works end-to-end" milestone. Research across K-Scale's ecosystem, ROS2 humanoid projects, sim-to-real pipelines, and practical robotics practice points to a clear path: **harden what we have** (calibration, safety, diagnostics), **unlock manipulation** (MoveIt2, gravity compensation, teleoperation), **stand up simulation** (mujoco_ros2_control with K-Scale's MJCF models), and **bridge to learning** (ksim for RL locomotion, teleoperation for imitation learning data collection). The hybrid approach used by PAL Robotics TALOS and the G1Pilot project — classical control for arms, RL for legs — is the most practical architecture for a KBot-class humanoid. Notable discovery: K-Scale Labs shut down in November 2025, but all code is open-sourced and the repos remain available.

---

## Phase 1: Harden and Complete (Now — before adding hardware)

### 1. Calibrate left arm zero offsets
- **What**: Use existing `read_offsets.py` to find zero offsets for motors 11-15 (left arm). Position each joint at zero pose, read raw encoder positions, compute offsets, update `steveros_ros2_control.xacro`. Follow the same procedure documented in `REPORT_zero_offset_calibration.md` for the right arm.
- **Why**: Without correct zeros, trajectory planning, gravity compensation, and collision detection are all wrong. 15 joints remain uncalibrated.
- **Source**: [K-Scale zeroing procedure](https://docs.kscale.dev/robots/k-bot/zeroing/) — same Robstride motors, same CAN protocol. Also our own proven procedure on motors 21-25.
- **Complexity**: SMALL (a few hours)
- **Depends on**: None

### 2. Add emergency stop to hardware interface
- **What**: Add an `is_estop_` flag to `steveros_hardware.cpp`. When triggered (via a ROS2 service `/estop`), set all motor torques to zero and ignore controller commands. Return `hardware_interface::return_type::ERROR` from `write()` to trigger the ros2_control lifecycle error cascade (controllers auto-deactivate per ros2_control PR #2334). Add a physical e-stop button via GPIO monitored by `estop-watchdog` node.
- **Why**: Consensus from the ros2_control SIG is that e-stop belongs in the hardware interface, not a separate controller. Required before any autonomous motion or adding higher-torque leg motors.
- **Source**: [ros-sig-robot-control e-stop discussion](https://groups.google.com/g/ros-sig-robot-control/c/dDIyo2pyaFg), [estop-watchdog](https://github.com/resibots/estop-watchdog), [ros2_control hardware component lifecycle](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
- **Complexity**: MEDIUM (a few days)
- **Depends on**: None

### 3. Add diagnostics to hardware interface
- **What**: Integrate `diagnostic_updater` into `steveros_hardware.cpp` to report on `/diagnostics`: CAN bus message rate and error frames, per-motor communication status (responding/timeout), torque proximity to limits, motor temperature (if available from Robstride feedback). Set up `rqt_robot_monitor` for visualization.
- **Why**: Need to know when things go wrong before adding complexity. Topic rate monitoring on `/joint_states` catches communication failures. Torque proximity warnings catch mechanical binding before damage.
- **Source**: [ROS2 diagnostics stack](https://github.com/ros/diagnostics), [ros2-system-monitor](https://github.com/AgoraRobotics/ros2-system-monitor)
- **Complexity**: MEDIUM (a few days)
- **Depends on**: None

### 4. Split into per-limb trajectory controllers
- **What**: Replace single `joint_trajectory_controller` with four instances: `right_arm_controller`, `left_arm_controller`, `right_leg_controller`, `left_leg_controller`. Keep single `controller_manager` and single `joint_state_broadcaster`. Each controller gets its own joint list and command/state interfaces. Set `allow_partial_joints_goal: true`.
- **Why**: Per-limb controllers allow independent motion of each limb, required for MoveIt2 planning groups and for operating arms while legs are offline. Standard pattern used by PAL TALOS and documented in ros2_control examples.
- **Source**: [ros2_control Example 13](https://control.ros.org/humble/doc/ros2_control_demos/example_13/doc/userdoc.html), PAL TALOS architecture, [MAB Robotics legged robots ROS2](https://mab-robotics.medium.com/legged-robots-ros2-6051f9c907cd)
- **Complexity**: SMALL (a few hours — config change, no code changes)
- **Depends on**: None

### 5. Set up torque monitoring baselines
- **What**: The hardware interface already exports effort (torque) state at `steveros_hardware.cpp:248`. Log effort values from `/joint_states` across all joints during typical motions. Establish per-joint baseline torque ranges. Add absolute torque thresholds (e.g., 80% of max) that trigger e-stop.
- **Why**: Foundation for collision detection. Simple threshold-based monitoring catches gross failures (motor stall, collision with environment) without needing a full dynamics model.
- **Source**: [Sensorless collision detection via current residuals](https://www.sciencedirect.com/science/article/abs/pii/S0736584524000632), [MoveIt2 sensorless collision detection discussion](https://discourse.ros.org/t/moveit-2-journey-sensorless-collision-detection-with-ros-2/9329)
- **Complexity**: SMALL (a few hours)
- **Depends on**: Step 2 (e-stop mechanism to trigger)

---

## Phase 2: Unlock Manipulation (1-3 weeks)

### 6. MoveIt2 configuration via Setup Assistant
- **What**: Run `moveit_setup_assistant` on the URDF to generate an SRDF. Define planning groups: `right_arm` (5 joints from torso to right wrist), `left_arm` (5 joints from torso to left wrist), `dual_arms` (super-group, solver=None). Generate the self-collision matrix (ACM). Define named poses (home, zero, reach). Produces a `steveros_moveit_config` package with launch files, SRDF, kinematics.yaml, and planning pipeline configs.
- **Why**: MoveIt2 is the standard ROS2 motion planning framework (126+ robots). Provides collision-free trajectory planning, inverse kinematics, and the foundation for teleoperation via MoveIt Servo. The SRDF collision matrix prevents self-collision without custom code.
- **Source**: [MoveIt2 Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html), [URDF/SRDF tutorial](https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)
- **Complexity**: MEDIUM (a few days)
- **Depends on**: Step 1 (calibrated joints), Step 4 (per-limb controllers)

### 7. MoveIt Servo for realtime arm teleoperation
- **What**: Configure MoveIt Servo for both arms. Accepts joint jog commands, end-effector twist (ideal for gamepad/SpaceMouse), and end-effector pose targets. Built-in singularity handling (velocity scaling near singularities) and self-collision avoidance. Connect a gamepad via `moveit_servo_gamepad_teleoperation`. Runs at realtime priority on RT kernels.
- **Why**: Teleoperation is the fastest path to useful arm behaviors and the foundation for collecting imitation learning demonstrations. MoveIt Servo handles the hard problems (singularity, collision) out of the box.
- **Source**: [MoveIt Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html), [gamepad teleoperation](https://moveit.picknik.ai/main/doc/how_to_guides/controller_teleoperation/controller_teleoperation.html)
- **Complexity**: MEDIUM (a few days)
- **Depends on**: Step 6 (MoveIt2 config)

### 8. Gravity compensation via torque feedforward
- **What**: Compute gravity torques using KDL (orocos_kdl) from the URDF kinematic chain. The Robstride MIT mode command frame already accepts `(position, velocity, Kp, Kd, torque_feedforward)` — add computed gravity torques as the feedforward term. Implement either as: (a) computation inside `steveros_hardware.cpp::write()`, or (b) a custom chained controller that outputs effort commands consumed by the hardware interface alongside position/velocity.
- **Why**: Without gravity compensation, arms drift downward under their own weight when holding poses. Matters for teleoperation quality, holding objects, and any manipulation task. The MIT mode torque feedforward slot is architecturally ideal for this.
- **Source**: [om_gravity_compensation_controller](https://docs.ros.org/en/rolling/p/om_gravity_compensation_controller/) (ROBOTIS, KDL-based), [PAL Robotics gravity compensation](https://docs.pal-robotics.com/sdk-dev/hardware/controllers/gravity-compensation), [chained controllers (Example 12)](https://control.ros.org/humble/doc/ros2_control_demos/example_12/doc/userdoc.html)
- **Complexity**: MEDIUM-LARGE (requires accurate URDF inertial parameters: mass, CoM, inertia tensors)
- **Depends on**: Step 1 (calibrated joints), accurate URDF inertials

### 9. IMU integration (torso-mounted)
- **What**: Mount a Bosch BNO055 on the torso link. Use [flynneva/bno055](https://github.com/flynneva/bno055) ROS2 driver (I2C or UART). Publishes `sensor_msgs/msg/Imu` with on-chip sensor fusion at 100Hz. Add to URDF as a sensor link. Optionally integrate with `robot_localization` EKF for smooth state estimation.
- **Why**: Required for bipedal balance (RL policies expect gravity vector from IMU), body orientation feedback, and fall detection. BNO055 is the cheapest path (~$30) with on-chip fusion. Upgrade to MicroStrain CV7 (~$1000) for serious bipedal work.
- **Source**: [flynneva/bno055 ROS2 driver](https://github.com/flynneva/bno055), [REP-145 IMU conventions](https://www.ros.org/reps/rep-0145.html), [Duke Humanoid (MicroStrain CV7)](https://arxiv.org/html/2409.19795), [robot_localization](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
- **Complexity**: MEDIUM (a few days — hardware mounting + driver config + TF integration)
- **Depends on**: None (can proceed in parallel)

---

## Phase 3: Simulation and Legs (2-6 weeks)

### 10. MuJoCo simulation via mujoco_ros2_control
- **What**: Install `ros-controls/mujoco_ros2_control` (NASA JSC origin, ros-controls maintained). Convert URDF to MJCF using K-Scale's [urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf), or use [dfki-ric/mujoco_ros2_control](https://github.com/dfki-ric/mujoco_ros2_control) which accepts URDF directly. Configure Robstride actuator parameters (`ctrlrange`, `forcerange`, `gainprm`, `biasprm`) matching verified motor parameter ranges. Cross-reference against K-Scale's official KBot MJCF model (`ks robots urdf mujoco kbot-v1`).
- **Why**: Simulation enables testing controllers, trajectories, and eventually RL policies without risking hardware. The mujoco_ros2_control plugin swaps in as a `SystemInterface` — existing launch files, controller configs, and URDF transfer directly. Published in JOSS.
- **Source**: [ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control), [JOSS paper](https://joss.theoj.org/papers/10.21105/joss.09140), [dfki-ric/mujoco_ros2_control](https://github.com/dfki-ric/mujoco_ros2_control), [urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf), [simulator integrations](https://control.ros.org/rolling/doc/simulators/simulators.html)
- **Complexity**: LARGE (a week+ — MJCF model creation/validation, actuator parameter tuning, sim-real matching)
- **Depends on**: Step 1 (calibrated joints for validation)

### 11. Wire and bring up legs on test stand
- **What**: Wire leg CAN bus connections. Verify motor communication with `read_offsets.py`. Calibrate zero offsets for all 10 leg joints (5 per leg). Test single leg on a gantry/test stand (never free-standing initially). Follow standard progression: single joint -> single leg -> both legs -> both legs with IMU feedback.
- **Why**: Legs have fundamentally different requirements: body weight support, impact forces, higher torques. Testing on a stand eliminates fall risk during bringup. Universal practice — PAL, Unitree, Asimov, K-Scale all use gantries for initial leg testing.
- **Source**: [Asimov leg bringup](https://news.asimov.inc/p/how-we-built-humanoid-legs-from-the), [IMSystems humanoid leg challenges](https://imsystems.nl/challenges-in-building-humanoid-joints-part-1-legs-balance/), K-Scale quickstart (gantry for initial standing)
- **Complexity**: LARGE (a week+ — wiring, calibration, test stand setup, tuning)
- **Depends on**: Step 2 (e-stop — critical for high-torque leg motors), Step 4 (per-limb controllers)

### 12. Servo tuning with ktune methodology
- **What**: Use K-Scale's [ktune](https://github.com/kscalelabs/ktune) or implement equivalent sine wave, step response, and chirp tests to compare simulated vs real motor response. Tune MJCF actuator parameters until sim matches real hardware. Closes the sim-to-real gap for future RL policy transfer.
- **Why**: Sim-to-real gap is the #1 reason RL policies fail on real hardware. ktune was developed specifically for matching Robstride motor dynamics between simulation and reality.
- **Source**: [ktune](https://github.com/kscalelabs/ktune)
- **Complexity**: MEDIUM (a few days)
- **Depends on**: Step 10 (simulation working)

---

## Phase 4: Autonomy and Learning (1-3 months)

### 13. RL locomotion training with ksim
- **What**: Set up [ksim](https://github.com/kscalelabs/ksim) and [ksim-kbot](https://github.com/kscalelabs/ksim-kbot) for PPO locomotion policy training. Train in MuJoCo MJX with JAX (8192 parallel environments on GPU). Walking policies converge in ~30 minutes on RTX 4090. Export trained policy to ONNX (bypassing K-Scale's `.kinfer` format) and wrap in a ROS2 controller node.
- **Why**: Canonical training pipeline for this exact robot. K-Scale validated walking on real KBot hardware. Code is MIT-licensed and fully available despite K-Scale's shutdown.
- **Source**: [ksim](https://github.com/kscalelabs/ksim), [ksim-kbot](https://github.com/kscalelabs/ksim-kbot), [ksim-gym](https://github.com/kscalelabs/ksim-gym), [kbot-joystick](https://github.com/kscalelabs/kbot-joystick)
- **Complexity**: LARGE (a week+ — environment setup, training infra, policy deployment node, sim-to-real tuning)
- **Depends on**: Step 10 (simulation), Step 11 (legs wired), Step 12 (sim-real matching)

### 14. ONNX policy deployment node
- **What**: Write a ROS2 C++ node that loads a trained ONNX model via `onnxruntime` C++ API. At each control tick (~50-200Hz), read `/joint_states`, construct input tensor (joint positions, velocities, IMU gravity vector, phase variable), run `session.Run()` (~1ms GPU, ~7.5ms CPU), publish joint commands to per-limb trajectory controllers. CPU execution provider initially; add CUDA if needed.
- **Why**: Bridges K-Scale's training stack (JAX/MuJoCo) to the ROS2 control stack. ONNX is the standard interchange format.
- **Source**: [ONNX Runtime C++ guide](https://onnxruntime.ai/docs/get-started/with-cpp.html), [Lei Mao ONNX example](https://github.com/leimao/ONNX-Runtime-Inference), [Isaac Lab policy deployment](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/isaac_lab_tutorials/tutorial_policy_deployment.html)
- **Complexity**: MEDIUM (a few days for the node; tuning for real hardware is ongoing)
- **Depends on**: Step 13 (trained policy)

### 15. Teleoperation for imitation learning data collection
- **What**: Build GELLO-style leader arms (~$300/arm) — 3D-print kinematically-scaled replicas of KBot arm chain with Dynamixel XL-330R servos. Map leader joint angles to follower (robot) joint commands. Record demonstrations as joint trajectories. Alternatively, use Meta Quest 3 VR with hand tracking and IK retargeting (faster setup, lower demo quality).
- **Why**: High-quality demonstrations are the foundation for imitation learning. GELLO outperforms VR controllers and SpaceMouse in user studies. Data feeds directly into frameworks like LeRobot.
- **Source**: [GELLO project](https://wuphilipp.github.io/gello_site/), [GELLO paper](https://arxiv.org/abs/2309.13037), [GELLO software](https://github.com/wuphilipp/gello_software), [Open-TeleVision (VR)](https://robot-tv.github.io/), [Unitree IL LeRobot](https://github.com/unitreerobotics/unitree_IL_lerobot)
- **Complexity**: LARGE (custom hardware design for KBot arm kinematics)
- **Depends on**: Step 7 (MoveIt Servo for safety during teleoperation)

### 16. Whole-body control framework
- **What**: Integrate Task Space Inverse Dynamics (TSID) via [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid) (Pinocchio for rigid body dynamics). Define task hierarchy: (1) self-collision avoidance, (2) CoM stabilizer, (3) foot contact constraints, (4) arm end-effector tasks. Implement as a ros2_control controller outputting joint torques.
- **Why**: Enables simultaneous walking + reaching. PAL TALOS uses exactly this architecture. Bridges independent limb control and coordinated full-body behavior.
- **Source**: [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid), [PAL WBC utils](https://github.com/pal-robotics/pal_wbc_utils), [Benchmarking WBC on TALOS](https://www.frontiersin.org/articles/10.3389/frobt.2022.826491/full)
- **Complexity**: LARGE (a week+ — dynamics model, task definition, QP solver tuning)
- **Depends on**: Steps 8 (gravity comp), 9 (IMU), 11 (legs)

---

## Key Discoveries

1. **K-Scale Labs shut down in November 2025** after failing to secure production funding. All IP was open-sourced under MIT/CERN-OHL-S-2.0. Only 2 KBots shipped. Repos remain live and usable. Successor venture "Gradient Robots" was founded by the COO but details are sparse. This means no upstream maintenance — fork what you need.

2. **K-Scale did NOT use ROS2.** Their entire stack is Rust-based (K-OS) with gRPC. The valuable K-Scale artifacts for this project are: validated motor gains, MuJoCo models, zeroing procedures, and the RL training pipeline (ksim). Not the runtime.

3. **The hybrid approach is dominant.** Both G1Pilot (Unitree G1) and PAL TALOS use classical control for arms (MoveIt2, trajectory controllers) and either RL or specialized locomotion controllers for legs. Nobody runs pure MoveIt2 planning on legs.

4. **Three competing MuJoCo-ros2_control bridges exist.** The ros-controls version (NASA JSC origin) is most actively maintained and JOSS-published. The DFKI version can use URDF directly without MJCF conversion. The MoveIt version exists but is less maintained.

5. **MIT mode command frame is ideal for gravity compensation.** The `(pos, vel, Kp, Kd, torque_ff)` structure means gravity torques slot directly into the feedforward term without switching control modes. Significant architectural advantage over pure position-command systems.

6. **RoboParty ROBOTO ORIGIN** (Xiaomi-backed, open-sourced January 2026, 1000+ GitHub stars) is a new and highly relevant reference project — 1.25m humanoid with full ROS2 Humble + IsaacLab + MuJoCo pipeline, GPL v3.

---

## Sources

1. [K-Scale KBot zeroing procedure](https://docs.kscale.dev/robots/k-bot/zeroing/)
2. [ros-controls/mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)
3. [dfki-ric/mujoco_ros2_control](https://github.com/dfki-ric/mujoco_ros2_control)
4. [MujocoROS2Control JOSS paper](https://joss.theoj.org/papers/10.21105/joss.09140)
5. [ksim (K-Scale RL training)](https://github.com/kscalelabs/ksim)
6. [ksim-kbot (KBot policies)](https://github.com/kscalelabs/ksim-kbot)
7. [kbot-joystick](https://github.com/kscalelabs/kbot-joystick)
8. [urdf2mjcf](https://github.com/kscalelabs/urdf2mjcf)
9. [ktune (servo tuning)](https://github.com/kscalelabs/ktune)
10. [kinfer-sim](https://github.com/kscalelabs/kinfer-sim)
11. [MoveIt2 Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
12. [MoveIt Servo tutorial](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
13. [MoveIt Servo gamepad teleoperation](https://moveit.picknik.ai/main/doc/how_to_guides/controller_teleoperation/controller_teleoperation.html)
14. [om_gravity_compensation_controller](https://docs.ros.org/en/rolling/p/om_gravity_compensation_controller/)
15. [PAL Robotics gravity compensation](https://docs.pal-robotics.com/sdk-dev/hardware/controllers/gravity-compensation)
16. [stack-of-tasks/tsid](https://github.com/stack-of-tasks/tsid)
17. [PAL WBC utils](https://github.com/pal-robotics/pal_wbc_utils)
18. [Benchmarking WBC on TALOS](https://www.frontiersin.org/articles/10.3389/frobt.2022.826491/full)
19. [ros2_control e-stop SIG discussion](https://groups.google.com/g/ros-sig-robot-control/c/dDIyo2pyaFg)
20. [estop-watchdog](https://github.com/resibots/estop-watchdog)
21. [ros2_control hardware component docs](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
22. [ROS2 diagnostics stack](https://github.com/ros/diagnostics)
23. [flynneva/bno055 ROS2 driver](https://github.com/flynneva/bno055)
24. [Duke Humanoid (MicroStrain CV7)](https://arxiv.org/html/2409.19795)
25. [robot_localization](https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/)
26. [ONNX Runtime C++ guide](https://onnxruntime.ai/docs/get-started/with-cpp.html)
27. [GELLO project](https://wuphilipp.github.io/gello_site/)
28. [GELLO paper](https://arxiv.org/abs/2309.13037)
29. [Open-TeleVision](https://robot-tv.github.io/)
30. [Humanoid-Gym (zero-shot sim2real)](https://github.com/roboterax/humanoid-gym)
31. [MuJoCo Playground](https://playground.mujoco.org/)
32. [Isaac Lab](https://github.com/isaac-sim/IsaacLab)
33. [RoboParty ROBOTO ORIGIN](https://github.com/Roboparty/roboto_origin)
34. [G1Pilot (Unitree G1 ROS2)](https://github.com/hucebot/g1pilot)
35. [open-rdc Walking Pattern Generator](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator)
36. [ros2_control Example 12 (chained controllers)](https://control.ros.org/humble/doc/ros2_control_demos/example_12/doc/userdoc.html)
37. [ros2_control Example 13 (multi-robot)](https://control.ros.org/humble/doc/ros2_control_demos/example_13/doc/userdoc.html)
38. [K-Scale Labs shutdown report](https://mikekalil.com/blog/k-scale-labs-shuts-down/)
39. [Sensorless collision detection](https://www.sciencedirect.com/science/article/abs/pii/S0736584524000632)
40. [ros2_control simulator integrations](https://control.ros.org/rolling/doc/simulators/simulators.html)
41. [Unitree IL LeRobot](https://github.com/unitreerobotics/unitree_IL_lerobot)
42. [CHILD teleoperation](https://arxiv.org/html/2508.00162v1)
43. [Humanoid Locomotion and Manipulation Survey](https://arxiv.org/abs/2501.02116)
44. [Asimov leg bringup](https://news.asimov.inc/p/how-we-built-humanoid-legs-from-the)
45. [IMSystems humanoid leg challenges](https://imsystems.nl/challenges-in-building-humanoid-joints-part-1-legs-balance/)
