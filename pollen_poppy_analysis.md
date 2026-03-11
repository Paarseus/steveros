# Detailed Analysis: Pollen Robotics (Reachy) & Poppy Humanoid

**Date**: 2026-03-09 | **Analyst**: Claude Opus 4.6

---

## Overview

This report provides a deep-dive analysis of three open-source robotics projects from the Pollen Robotics ecosystem and the Poppy project. Each is evaluated across 9 criteria on a 1-10 scale with specific justifications referencing actual files and code patterns.

| Project | Repository | Stars | Forks | Contributors | Last Commit | Overall |
|---------|-----------|-------|-------|-------------|-------------|---------|
| Reachy 2 SDK | pollen-robotics/reachy2-sdk | 50 | 8 | 9 | Nov 2025 | **7/10** |
| Reachy Mini | pollen-robotics/reachy_mini | 997 | 180 | 30 | Mar 9, 2026 (today) | **8/10** |
| Poppy Humanoid | poppy-project/poppy-humanoid | 877 | 261 | 10 | Dec 2021 | **4/10** |

---

## Project 1: Reachy 2 SDK -- Overall: 7/10

**Repository:** https://github.com/pollen-robotics/reachy2-sdk
**License:** Apache-2.0 | **Language:** Python | **Created:** Aug 2023

A Python SDK for remote control of the full-size Reachy 2 humanoid robot via gRPC. Provides high-level API for arm kinematics, head control, mobile base, grippers, cameras, and audio.

### 1. Code Organization -- 8/10

**Directory structure** (`src/reachy2_sdk/`):
- `components/` -- Low-level actuator abstractions (antenna, goto-based component)
- `config/` -- Robot configuration discovery (`reachy_info.py`)
- `dynamixel/` -- Dynamixel motor wrappers
- `grippers/` -- Gripper joint + parallel gripper
- `media/` -- Audio, camera, camera manager
- `orbita/` -- Orbita 2D/3D actuator abstractions (7 files: `orbita.py`, `orbita2d.py`, `orbita3d.py`, `orbita_axis.py`, `orbita_joint.py`, `orbita_motor.py`, `utils.py`)
- `parts/` -- High-level robot parts: `arm.py`, `head.py`, `hand.py`, `mobile_base.py`, `tripod.py`, `part.py`, `joints_based_part.py`, `goto_based_part.py`
- `sensors/` -- Lidar sensor
- `utils/` -- Custom dict, goto helpers, utilities
- `reachy_sdk.py` -- Main entry point class

**Strengths:**
- Clean separation between low-level actuators (`orbita/`, `dynamixel/`) and high-level robot parts (`parts/`).
- The `Arm` class uses sophisticated dual inheritance from `JointsBasedPart` and `IGoToBasedPart`, providing both joint-level and Cartesian-level control.
- `reachy_sdk.py` aggregates all subsystems via gRPC stubs from the `reachy2_sdk_api` protobuf package (imports: `reachy_pb2`, `arm_pb2`, `goto_pb2`, `hand_pb2`, `head_pb2`).
- Thread-safe connection management with proper gRPC channel lifecycle.

**Weaknesses:**
- The `orbita/` package has 7 files for one actuator concept -- over-abstracted.
- No abstract base class separating SDK logic from gRPC transport.

### 2. Documentation -- 6/10

- README covers installation, examples, logging, testing; includes CI badges.
- 7 Jupyter notebooks in `src/examples/` (getting started through audio).
- 3 Python example scripts: `cameras.py`, `draw_square.py`, `set_default_posture.py`.
- pdoc-generated API docs on GitHub Pages; Google-style docstrings enforced.

**Weaknesses:** No architecture diagram. No contribution guide. Limited inline comments in core files.

### 3. Build System -- 6/10

- `pyproject.toml` (minimal: build-system, isort, black) + `setup.cfg` (dependencies, flake8, mypy, pytest, coverage).
- Dependencies: numpy 1.24-1.26, protobuf 4.25-5.30, grpcio 1.59-1.70, pyquaternion, opencv-python, reachy2-sdk-api.

**Weaknesses:** Split config across two files (modern projects use pyproject.toml only). No lockfile. No Docker. Uses older setuptools pattern.

### 4. Hardware Abstraction -- 6/10

- gRPC-based network abstraction provides inherent decoupling.
- Test infrastructure distinguishes "online" (robot required) from "offline" (pure unit) tests.
- FK/IK computed server-side via gRPC -- no local kinematics capability.

**Weaknesses:** No built-in simulation mode. Cannot test SDK logic without mocking the entire gRPC server.

### 5. Control Architecture -- 7/10

- `goto()` supporting joint-space and Cartesian-space movements.
- Interpolation modes: `minimum_jerk`, `linear`, `elliptical` (with arc directions: above, below, front, back, left, right).
- `turn_off_smoothly()` provides 3-second torque ramp-down.
- Motor-level torque/speed limit validation.
- Pose manipulation: `translate_by()`, `rotate_by()` in "robot" and "gripper" frames.

**Weaknesses:** Server-side kinematics only. No real-time control loop. No collision avoidance.

### 6. Community & Maintenance -- 5/10

- 9 contributors; lead developer (glannuzel) has 1,010 of ~1,800 commits (56%).
- Last commit: Nov 7, 2025 (4 months stale). 28 open issues.
- Clean PR workflow visible. Activity shifted to Reachy Mini.

**Weaknesses:** High bus factor risk. Slowing activity suggests product transition.

### 7. Extensibility -- 7/10

- Part-based architecture: subclass `JointsBasedPart` or `Part` to add body parts.
- GoTo-based component pattern for reusable motion primitives.
- gRPC API enables multi-language SDK implementations.
- `CustomDict` allows attribute-style and bracket-style joint access.

**Weaknesses:** Orbita hierarchy (7 files) is complex. No plugin system. Tightly coupled to Reachy 2 specifically.

### 8. Testing -- 7/10

- **19 offline unit tests**: arm, antenna, camera, custom_dict, dynamixel_motor, gripper_joint, head, lidar, mobile_base, orbita2d, orbita3d, orbita_axis, orbita_motor, orbita_utils, parallel_gripper, reachy, reachy_sdk, tripod, utils.
- **14 online integration tests**: actuator properties, advanced goto, antenna movements, audio, audit, basic movements, camera, hand, interpolations, mobile base, multiple connections, send goal positions, tripod.
- pytest markers: `online`, `offline`, `camera`, `audio`, `mobile_base`.
- CI runs offline tests automatically.

**Weaknesses:** No coverage tracking. Online tests require hardware. `test_arm.py` is one monolithic function.

### 9. CI/CD -- 7/10

- **5 workflows**: `lint.yml` (Black + isort + flake8 + mypy), `unit_tests.yml`, `docs.yml`, `publish_pypi.yml`, `release_to_notion.yaml`.
- 4-tool lint pipeline on every push.
- Auto-generated docs on GitHub Pages. PyPI publishing.

**Weaknesses:** Ubuntu-only CI. No coverage reporting. Pre-commit hooks exist as scripts but are not auto-installed.

---

## Project 2: Reachy Mini -- Overall: 8/10

**Repository:** https://github.com/pollen-robotics/reachy_mini
**License:** Apache-2.0 (software) + CC BY-SA-NC (hardware) | **Language:** Python | **Created:** May 2025

A **full-stack** open-source expressive robot platform: daemon server, Python SDK, hardware drivers, MuJoCo simulation, 3 kinematics engines, media (camera + audio + WebRTC), HuggingFace app store, web dashboard, and motor tools.

Three hardware variants: **Wireless** (autonomous, RPi4), **Lite** (USB-connected), **Simulation** (MuJoCo, no hardware).

### 1. Code Organization -- 9/10

**Directory structure** (`src/reachy_mini/`):
- `apps/` -- App store, `assistant.py` CLI, HF Space integration (`sources/hf_space.py`, `sources/local_common_venv.py`), Jinja2 templates
- `daemon/app/` -- FastAPI daemon: 13 REST API routers (`apps.py`, `daemon.py`, `kinematics.py`, `motors.py`, `move.py`, `state.py`, `volume.py`, `wifi_config.py`, `sdk_ws.py`, `logs.py`, `cache.py`, `hf_auth.py`, `update.py`)
- `daemon/app/dashboard/` -- Web UI (HTML/CSS/JS with SVG robot animations)
- `daemon/backend/abstract.py` -- ~1,000-line abstract backend defining complete robot interface
- `daemon/backend/robot/` -- Real hardware backend (serial/USB)
- `daemon/backend/mujoco/` -- MuJoCo simulation backend
- `daemon/backend/mockup_sim/` -- Lightweight mock for testing
- `descriptions/reachy_mini/mjcf/` -- MuJoCo model files (reachy_mini.xml, scene.xml, joints_properties.xml, empty.xml, minimal.xml)
- `descriptions/reachy_mini/urdf/` -- URDF (robot.urdf, robot_no_collision.urdf, robot_simple_collision.urdf)
- `io/` -- `abstract.py` (AbstractServer, AbstractClient), `protocol.py`, `publisher.py`, `ws_client.py`, `ws_server.py`
- `kinematics/` -- 3 engines: `analytical_kinematics.py` (Rust FFI via reachy-mini-rust-kinematics), `nn_kinematics.py` (ONNX), `placo_kinematics.py`
- `media/` -- Camera (camera_opencv.py, camera_gstreamer.py), Audio (audio_sounddevice.py, audio_gstreamer.py), WebRTC (webrtc_client_gstreamer.py, webrtc_daemon.py)
- `motion/` -- `goto.py`, `move.py`, `recorded_move.py`
- `tools/` -- Motor flashing/scanning/setup (7 tools), camera calibration suite
- `utils/` -- Constants, discovery, hardware config parser, interpolation, wireless version management
- `reachy_mini.py` -- Main SDK entry point

**Strengths:**
- Textbook separation of concerns across 10+ subpackages.
- `io/abstract.py` defines `AbstractServer` and `AbstractClient` with typed signatures: `get_current_joints() -> tuple[list[float], list[float]]`, `get_current_head_pose() -> npt.NDArray[np.float64]`, `send_command(cmd: AnyCommand)`.
- Three independent kinematics engines selectable at daemon startup.
- Platform-specific volume control: `volume_control_linux.py`, `volume_control_macos.py`, `volume_control_windows.py`.

**Weaknesses:** Monorepo size (daemon + SDK + hardware + dashboard in one package). 13 API routers suggest the daemon may be over-scoped.

### 2. Documentation -- 9/10

- **AGENTS.md** (200+ lines): AI-agent development guide covering robot basics, SDK essentials, dual motion paradigm (`goto_target` vs `set_target`), safety limits table (head pitch/roll +/-40 deg, yaw +/-180 deg, body yaw +/-160 deg), platform compatibility matrix, 8 example apps with HF links, 12 skills reference files.
- `docs/source/` tree: SDK (quickstart, python-sdk, core-concept, integration, javascript-sdk, media-architecture), platforms (reachy_mini, reachy_mini_lite, simulation), troubleshooting.
- 2 Jupyter notebooks, 14+ documented example apps.
- API docs auto-generated via CI. Bug report issue template. Contributing guide.

**Weaknesses:** No architecture diagram. External HF Space links may go stale.

### 3. Build System -- 8/10

- Modern `pyproject.toml`: 8 optional dependency groups (`mujoco`, `nn_kinematics`, `placo_kinematics`, `rerun`, `wireless-version`, `sounddevice`, `opencv`, `examples`) plus `[all]`.
- `uv` package manager with `uv.lock` for reproducible builds.
- Pre-commit hooks (`.pre-commit-config.yaml`), Ruff 0.12.0 with `extend-select = ["I", "D"]`, mypy 1.18.2 strict.
- 3 CLI entry points: `reachy-mini-daemon`, `reachy-mini-app-assistant`, `reachy-mini-reflash-motors`.
- `uv-lock-check.yml` CI workflow prevents lockfile drift.

**Weaknesses:** Still uses setuptools backend. No Docker dev environment.

### 4. Hardware Abstraction -- 9/10

- **Abstract backend pattern**: `daemon/backend/abstract.py` (~1,000 lines) defines the complete interface. Three implementations:
  - `robot/backend.py` -- Real hardware (serial/USB motors via `rustypot` + `reachy_mini_motor_controller`)
  - `mujoco/backend.py` -- Physics simulation (MuJoCo 3.3.0, headless via `MUJOCO_GL=disable`)
  - `mockup_sim/backend.py` -- Lightweight testing mock
- Daemon CLI (`--sim`, `--lite`, `--wireless`) selects backend. SDK connects via WebSocket, unaware of backend type.
- Camera abstraction: `camera_base.py` -> `camera_opencv.py`, `camera_gstreamer.py`.
- Audio abstraction: `audio_base.py` -> `audio_sounddevice.py`, `audio_gstreamer.py`.
- 3 URDF variants + MJCF model bundled in-repo.

### 5. Control Architecture -- 8/10

- Dual motion: `goto_target()` (smooth interpolation, 0.5s+) and `set_target()` (real-time, 10Hz+).
- 4 interpolation modes: `linear`, `minjerk`, `ease_in_out`, `cartoon`.
- Stewart platform 6-DOF head (x, y, z, roll, pitch, yaw).
- Automatic body yaw compensation (65 deg relative limit, 160 deg absolute).
- Gravity compensation mode. Thread-safe move execution with reentrant locking.
- Gaze control: `look_at_image()` (pixel coords), `look_at_world()` (3D coords).
- Movement recording/playback with HuggingFace dataset integration.

**Weaknesses:** No explicit real-time guarantees. Soft safety limits (SDK-side clamping). No collision avoidance.

### 6. Community & Maintenance -- 9/10

- **30 contributors** -- largest of any project analyzed.
- **997 stars, 180 forks**. Last commit: today (Mar 9, 2026). v1.5.1 released today.
- 112 open issues, 26 open PRs -- high activity.
- Discord community. HuggingFace app ecosystem.
- Commercial backing from Pollen Robotics.

### 7. Extensibility -- 8/10

- **App store**: `reachy-mini-app-assistant create <name> <path> --publish` -> HuggingFace Spaces discovery.
- Templates: default (minimal) and conversation (LLM integration).
- Pluggable kinematics: analytical (Rust), NN (ONNX), placo -- selected via daemon CLI.
- Pluggable media: OpenCV vs GStreamer for video, sounddevice vs GStreamer for audio.
- REST API + WebSocket SDK for any-language clients. JavaScript SDK documented.
- Rerun visualization as optional dependency.

**Weaknesses:** Tightly coupled to Reachy Mini form factor. Python-only app ecosystem currently.

### 8. Testing -- 7/10

- **12 unit tests**: analytical_kinematics, placo, app, audio, video, collision, daemon, discovery, import, js_protocol_sync, volume_control, wireless.
- **7 integration tests**: body yaw, kinematics comparison, recording comparison, gravity compensation, GStreamer, tracking, head position GUI.
- Fixture apps (`faulty_app/`, `ok_app/`) with own `pyproject.toml` for lifecycle testing.
- **Physical CI**: `reachy_mini_physical_ci.yml` runs on actual hardware across Linux/macOS/Windows.
- pytest-asyncio for async tests. `MUJOCO_GL=disable` for headless CI.

**Weaknesses:** No coverage tracking. Kinematics tests are basic (2 functions, identity pose only). Many tests marker-excluded in CI.

### 9. CI/CD -- 9/10

- **8 GitHub Actions workflows**:
  - `pytest.yml` -- Ubuntu + macOS, uv, Python 3.12, 10-min timeout
  - `lint.yml` -- Ruff
  - `reachy_mini_physical_ci.yml` -- Physical hardware across 3 OSes (manual dispatch)
  - `build_documentation.yml`, `build_pr_documentation.yml`, `upload_pr_documentation.yml`
  - `uv-lock-check.yml` -- Lockfile integrity
  - `wheels.yml` -- Package building
- Multi-platform. Pre-commit hooks. Path-based triggers.

**Weaknesses:** Physical CI is manual-dispatch. No coverage reporting.

---

## Project 3: Poppy Humanoid -- Overall: 4/10

**Repository:** https://github.com/poppy-project/poppy-humanoid
**License:** GPL v3 (software) + CC BY-SA (hardware) | **Language:** Jupyter Notebook | **Created:** Jan 2015

A historically significant open-source 3D-printed humanoid robot, founded at INRIA Bordeaux in 2012 (Flowers lab, ERC Grant Explorer). Pioneered affordable humanoid robotics for education and research. **Dormant since December 2021.**

### 1. Code Organization -- 4/10

**Directory structure:**
- `hardware/` -- STL/ (3D-printable parts), URDF/ (model + meshes + ROS 1 launch), head/, lower_limbs/, torso/, upper_limbs/ (Solidworks submodules)
- `software/poppy_humanoid/` -- `poppy_humanoid.py` (main class, ~80 lines), `configuration/poppy_humanoid.json` (motor config), `primitives/` (5 behavior files), `media/sounds/` (error.wav), `utils/` (empty `__init__.py`), `vrep-scene/` (V-REP scene)
- `software/samples/notebooks/` -- 5 Jupyter notebooks

**Strengths:** Clean hardware/software separation. Primitives pattern is elegant. Declarative JSON motor config (30+ motors).

**Weaknesses:** Main class is ~80 lines delegating everything to pypot. Empty `utils/`. ROS 1 launch files. Git submodules for CAD.

### 2. Documentation -- 5/10

- README: cost ($8,000-9,000), assembly time (7 hours), RPi3/4, pip install.
- 5 Jupyter notebooks. LaTeX software guide (PDF).
- Bill of materials and assembly instructions referenced.

**Weaknesses:** V-REP references (discontinued 2019). No API docs. Stale external links. No architecture overview.

### 3. Build System -- 3/10

- Legacy `setup.py` with single dependency: `pypot >= 4.0.0`.
- No `pyproject.toml`, no lockfile, no linting config, no pre-commit hooks.

### 4. Hardware Abstraction -- 5/10

- `pypot` library (external) provides actual hardware abstraction via `AbstractPoppyCreature`.
- `poppy_humanoid.json`: 30+ motors (MX-28, MX-64 Dynamixel), IDs 11-54, angle limits, offsets, 2 controller groups.
- V-REP simulation via `vrep-scene/poppy_humanoid.ttt`. `robot.simulated` flag for conditional behavior.
- Gazebo support via URDF + ROS 1 launch files.
- `vrep_hack()` manually fixes motor orientation for 3 joints (`r_knee_y`, `abs_x`, `bust_x`) -- fragile workaround.

**Weaknesses:** V-REP discontinued. All abstraction in external pypot. `add_vrep_methods()` uses raw ctypes.

### 5. Control Architecture -- 4/10

- **Primitives pattern**: `pypot.primitive.Primitive` (one-shot) and `LoopPrimitive` (periodic).
- `LimitTorque` (safe.py): Dynamic torque scaling based on position error -- thoughtful.
- `TemperatureMonitor` (safe.py): Cross-platform audio alerts (aplay/afplay/wmplayer).
- `StandPosition`/`SitPosition` (posture.py): Hardcoded angle dictionaries with `goto_position()`.
- `InitRobot`: PID `(4, 2, 0)` for MX, `(6, 2, 0)` for torso. Torque limit 70%.
- Primitive mixing: `filter = partial(numpy.sum, axis=0)` sums simultaneous outputs.

**Weaknesses:** No kinematics in codebase. Hardcoded magic numbers everywhere. No trajectory planning. No real-time guarantees. Safety is reactive only.

### 6. Community & Maintenance -- 2/10

- Last commit: Dec 6, 2021 (4+ years ago). 10 contributors, zero activity since 2021.
- 5 open issues, no responses. Forum declining.
- 877 stars/261 forks reflect historical importance, not vitality.

**Assessment:** Effectively abandoned. Team moved to Reachy 2 and Reachy Mini.

### 7. Extensibility -- 5/10

- Primitives are composable: subclass `Primitive`, implement `setup()/run()/teardown()`, attach via `robot.attach_primitive()`.
- JSON motor config makes hardware changes simple.
- Creature pattern designed for robot variants (Ergo Jr, Torso, etc.).

**Weaknesses:** No plugin system/app store. Sensor changes require pypot modification. V-REP-only sim.

### 8. Testing -- 1/10

- **Zero tests.** No pytest, unittest, or any framework. No test directory exists.

### 9. CI/CD -- 1/10

- **No CI/CD of any kind.** No GitHub Actions, no Travis CI. No `.github/workflows/`. No linting.

---

## Cross-Project Comparison Matrix

| Criterion | Reachy 2 SDK | Reachy Mini | Poppy Humanoid |
|-----------|:------------:|:-----------:|:--------------:|
| **1. Code Organization** | 8 | 9 | 4 |
| **2. Documentation** | 6 | 9 | 5 |
| **3. Build System** | 6 | 8 | 3 |
| **4. Hardware Abstraction** | 6 | 9 | 5 |
| **5. Control Architecture** | 7 | 8 | 4 |
| **6. Community & Maintenance** | 5 | 9 | 2 |
| **7. Extensibility** | 7 | 8 | 5 |
| **8. Testing** | 7 | 7 | 1 |
| **9. Overall (weighted avg)** | **7/10** | **8/10** | **4/10** |

---

## Evolutionary Analysis: The Pollen Robotics Trajectory

These three projects trace the evolution of a robotics company's software engineering maturity:

**Poppy Humanoid (2012-2021):** Pioneer era. Proved the concept of affordable open-source humanoids. Software was secondary to hardware innovation. No engineering discipline (zero tests, zero CI).

**Reachy 2 SDK (2023-2025):** Professional SDK era. Proper packaging, gRPC API, 33 test files, 4-tool CI pipeline. But still a client library, not a platform. High bus factor risk.

**Reachy Mini (2025-present):** Platform era. Full-stack with daemon architecture, 3 backends, app ecosystem, physical CI, AI-agent workflows, 30 contributors. Represents state-of-the-art open-source robotics software engineering.

Each generation addresses the previous one's weaknesses. Reachy Mini in particular fixes every major deficiency of Poppy Humanoid while adding capabilities that did not exist in the robotics open-source ecosystem before (HuggingFace app store, AGENTS.md, physical CI across 3 OSes).

---

## Key Takeaways for Steveros

1. **Reachy Mini's daemon/backend pattern** (`abstract.py` -> `robot/`, `mujoco/`, `mockup_sim/`) is the most directly applicable architecture pattern for Steveros's hardware abstraction improvement.

2. **AGENTS.md** is a forward-thinking practice. Steveros already has rich documentation in MEMORY.md -- structuring this for AI agents would accelerate development.

3. **Physical CI on real hardware** is the gold standard. Even manual-dispatch physical CI is better than none.

4. **Poppy's fate is a warning**: no tests + no CI = abandoned project. Steveros must invest in testing now while the project is active.

5. **The app ecosystem model** (Reachy Mini's HuggingFace integration) is a novel pattern for robot behavior distribution worth studying for Steveros's future.
