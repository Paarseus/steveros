# Open-Source Humanoid Robot Projects: Comprehensive Analysis (Wave 2)

**Compiled for SteveBots / Steveros** | March 2026

---

## Table of Contents

1. [Project 1: LearningHumanoidWalking](#1-learninghumanoidwalking)
2. [Project 2: LocoMuJoCo](#2-locomujoco)
3. [Project 3: OpenArm (Enactic)](#3-openarm-enactic)
4. [Project 4: Open X-Humanoid (TienKung-Lab + XR-1)](#4-open-x-humanoid)
5. [Comparative Summary](#5-comparative-summary)
6. [Recommendations for Steveros](#6-recommendations-for-steveros)

---

## 1. LearningHumanoidWalking

**Repository:** https://github.com/rohanpsingh/LearningHumanoidWalking
**Stars:** 1,073 | **Forks:** 125 | **License:** BSD-2-Clause
**Created:** July 2022 | **Last Commit:** March 8, 2026
**Contributors:** 1 (single-author project with PR-based workflow)

### Overview

RL-based locomotion training for humanoid robots, supporting JVRC and Unitree H1 platforms. Implements omnidirectional walking, stepping with footstep targets, and cartpole swing-up. Built on MuJoCo with PyTorch PPO.

### Directory Structure

```
LearningHumanoidWalking/
├── .github/workflows/     # CI: tests.yml + lint.yml
├── .githooks/             # Git hooks
├── envs/                  # Environment implementations
│   ├── common/            # BaseHumanoidEnv, MujocoEnv, domain_randomization
│   ├── h1/                # Unitree H1 environment
│   ├── jvrc/              # JVRC environment
│   └── cartpole/          # Cartpole swing-up
├── rl/                    # RL algorithms
│   ├── algos/ppo.py       # PPO implementation
│   ├── policies/          # Actor, Critic, base policy
│   ├── distributions/     # Action distributions
│   ├── storage/           # Rollout buffers
│   └── workers/           # Parallel training workers
├── robots/robot_base.py   # Robot abstraction
├── tasks/                 # Task definitions
│   ├── rewards.py         # Parameterized reward functions
│   ├── base_task.py       # Task interface
│   ├── standing_task.py
│   ├── walking_task.py
│   └── stepping_task.py
├── tests/                 # 5 test files, comprehensive
├── models/                # MuJoCo XML models
├── scripts/               # Utility scripts
├── utils/                 # Shared utilities
├── pyproject.toml         # Modern build config (uv + hatch)
├── .pre-commit-config.yaml
└── CLAUDE.md              # AI coding instructions
```

### Criterion Ratings

#### 1. Code Organization: 9/10

Exemplary separation of concerns. The codebase is organized into five clear domains: environments (`envs/`), RL algorithms (`rl/`), robot abstractions (`robots/`), task definitions (`tasks/`), and utilities. The `BaseHumanoidEnv` class in `envs/common/base_humanoid_env.py` is a textbook abstract base class with well-defined abstract methods (`_build_xml()`, `_setup_robot()`, `_get_robot_state()`, `_get_external_state()`). New robots only need to implement these four methods. The RL stack is completely decoupled from environment specifics -- `rl/algos/ppo.py`, `rl/policies/`, `rl/storage/` form a self-contained PPO implementation. Configuration is cleanly managed via YAML files in per-robot `configs/` directories.

Domain randomization (`domain_randomization.py`) and observation noise are cleanly separated from core logic. Action smoothing is parameterized as a config value. The `pyproject.toml` explicitly declares `packages = ["rl", "envs", "tasks", "robots"]`, showing intentional module boundaries.

Minor deduction: single `robot_base.py` file rather than a `robots/` package with per-robot implementations.

#### 2. Documentation: 7/10

The README provides clear installation (using modern `uv` tooling), training/evaluation commands, a directory structure overview, and instructions for adding custom robots. It references three peer-reviewed papers. However, there is no dedicated API documentation site, no docstring-generated docs, and no in-depth tutorials. The `CLAUDE.md` file provides AI-readable project context, which is unconventional but useful. The code itself has excellent docstrings (the `BaseHumanoidEnv` class docstring explains all abstract methods and their contracts).

#### 3. Build System: 9/10

Modern Python packaging with `pyproject.toml` using Hatchling build backend and `uv` package manager. CI consists of two workflows: `tests.yml` (runs pytest with slow/fast test separation, installs system dependencies for headless rendering via osmesa) and `lint.yml` (Ruff for linting and formatting). Pre-commit hooks are configured. Ruff config includes pycodestyle, Pyflakes, pyupgrade, flake8-bugbear, flake8-simplify, and isort -- a comprehensive lint setup. The `uv.lock` file ensures reproducible builds. Dependencies are precisely pinned (`mujoco==3.4.0`, `torch==2.5.1`).

#### 4. Hardware Abstraction: 6/10

The project is simulation-only (MuJoCo). There is a clean `RobotInterface` abstraction and the `BaseHumanoidEnv` provides a solid foundation for adding new robots via YAML configs and XML generation. However, there is no real-robot deployment path, no hardware interface layer, and no sim-to-real transfer tooling.

Two robots are supported: JVRC and Unitree H1, plus the cartpole benchmark.

#### 5. Control Architecture: 7/10

Solid RL control pipeline with PPO. Key design qualities:
- Physics stepping at 0.001s sim_dt with 0.025s control_dt (25:1 ratio)
- Action smoothing between timesteps
- Domain randomization (dynamics and perturbations) with configurable intervals
- Observation history buffering via `collections.deque` with configurable length
- Initialization noise for robustness
- Observation noise injection (uniform or Gaussian, per-observation-component scales)
- Mirror symmetry support for sample-efficient training
- Clock-based gait signals for locomotion tasks

#### 6. Community & Maintenance: 6/10

Single contributor with active development (most recent commit March 8, 2026). Uses a PR-based workflow (PRs #66-#76 visible). 18 open issues. 1,073 stars. Bus-factor risk from single contributor.

#### 7. Extensibility: 8/10

Documented 4-step process for adding custom robots. Abstract base class pattern with clear interface contracts. YAML-based configuration. `envs/__init__.py` registry enables auto-discovery.

#### 8. Testing: 9/10

Exceptional test quality. Five test files covering: parametrized environment tests (15+ tests auto-discovering all environments), training integration, determinism, evaluation, and shared fixtures. Tests split into fast/slow categories via pytest markers. CI runs both.

#### 9. Overall Rating: 7.6/10

A focused, well-engineered RL locomotion framework with excellent code quality and testing. Limited by being simulation-only with a single contributor.

---

## 2. LocoMuJoCo

**Repository:** https://github.com/robfiras/loco-mujoco
**Stars:** 1,344 | **Forks:** 139 | **License:** MIT
**Created:** June 2023 | **Last Commit:** May 30, 2025
**Contributors:** 11 | **Default Branch:** master

### Overview

Imitation learning benchmark for whole-body locomotion control. Features 16 environments (12 humanoid, 4 quadruped), dual MuJoCo/MJX backends, JIT-compiled JAX training loops, 22,000+ mocap samples, and implementations of PPO, GAIL, AMP, and DeepMimic algorithms. Published at NeurIPS 2023 Robot Learning Workshop.

### Directory Structure

```
loco-mujoco/
├── .github/workflows/continuous_integration.yml
├── docs/                           # Sphinx documentation
├── examples/
│   ├── training_examples/          # PPO, GAIL, AMP, DeepMimic
│   ├── tutorials/                  # 11 numbered tutorials
│   ├── replay_datasets/
│   ├── trajectory_generation/
│   └── speed_test.py
├── loco_mujoco/
│   ├── algorithms/                 # PPO, GAIL, AMP (single-file JAX)
│   ├── core/                       # Control, domain rand, observations, rewards, terminals, terrain, visuals, wrappers
│   ├── datasets/
│   ├── environments/
│   │   ├── humanoids/              # 12 robots
│   │   ├── quadrupeds/             # 4 robots
│   │   └── base.py
│   ├── models/
│   ├── smpl/
│   ├── task_factories/
│   ├── trajectory/
│   └── utils/
├── tests/                          # 13 test files
├── pyproject.toml
├── setup.py
├── .readthedocs.yaml
└── Makefile
```

### Criterion Ratings

#### 1. Code Organization: 8/10

Well-structured package. `core/` directory is particularly well-organized with subdirectories for `control_functions/`, `domain_randomizer/`, `initial_state_handler/`, `observations/`, `reward/`, `terminal_state_handler/`, `terrain/`, and `wrappers/`. Each robot has CPU and MJX variants. Minor deduction: dual `pyproject.toml` + `setup.py` and `numpy<2.0` pin with `mujoco==3.2.7`.

#### 2. Documentation: 9/10

Outstanding: ReadTheDocs site, 11 numbered tutorials, 4 training example directories, HuggingFace datasets, NeurIPS publication, usage examples in README.

#### 3. Build System: 5/10

Uses `setuptools`. CI runs on push to master only (not PRs), skips `test_task_factories.py`. Flake8 commented out. No pre-commit hooks. `mujoco==3.2.7` with TODO about breaking API changes.

#### 4. Hardware Abstraction: 7/10

Pure simulation but exceptional diversity: 12 humanoids (Atlas, Apollo, H1, H1v2, G1, GR1T2, BoosterT1, Talos, ToddlerBot, MyoSkeleton, skeletons), 4 quadrupeds (AnymalC, Spot, A1, Go2), SMPL biomechanical models. Dual MuJoCo/MJX backend. Cross-robot motion retargeting.

#### 5. Control Architecture: 8/10

JIT-compiled training loops (36 min for omnidirectional walking on RTX 3080 Ti). Composable modules for observations, rewards, terminals, control functions. PPO, GAIL, AMP, DeepMimic algorithms. DTW and Frechet distance trajectory analysis.

#### 6. Community & Maintenance: 7/10

11 contributors. Issue response: same-day to ~2 weeks. 38 open issues. 1,344 stars. NeurIPS publication. Last commit May 2025 (9+ months ago) suggests maintenance phase.

#### 7. Extensibility: 9/10

11 tutorials, task factory pattern, composable modules, Gymnasium interface, cross-robot retargeting, Hydra configuration.

#### 8. Testing: 7/10

13 test files covering algorithms, control, domain randomization, goals, initial states, observations, rewards, terminals, MJX, task factories, trajectories. Tests use `make_jaxpr` for compilation verification. CI skips `test_task_factories.py`.

#### 9. Overall Rating: 7.5/10

Most comprehensive benchmark platform surveyed. Exceptional robot diversity, algorithm coverage, documentation. Weakened by aging build tooling, incomplete CI, development slowdown.

---

## 3. OpenArm (Enactic)

**Repository (hub):** https://github.com/enactic/openarm
**Stars:** 1,897 (hub) | **Forks:** 200 | **License:** Apache-2.0
**Created:** September 2024 | **Last Commit:** March 9, 2026
**Contributors:** 22 (hub repo) + additional across sub-repos

### Overview

Open-source 7-DOF humanoid robotic arm designed for physical AI research. Multi-repository project spanning hardware (CAD, BOM), firmware (CAN bus), ROS2 integration, teleoperation (unilateral + bilateral), and Isaac Lab simulation. Cost: $6,500 USD for bimanual system. Backed by Enactic (Reazon Holdings).

### Sub-Repository Ecosystem

| Repository | Stars | Purpose | CI | Tests |
|---|---|---|---|---|
| openarm (hub) | 1,897 | Docs website, coordination | Lint, website deploy, dependabot | N/A |
| openarm_hardware | 310 | CAD (STL, STEP, Fusion360) | N/A | N/A |
| openarm_description | 22 | URDF/XACRO | N/A | N/A |
| openarm_can | 38 | C++ CAN communication | Lint + multi-platform build + Python test | Build tests, import test |
| openarm_ros2 | 83 | ROS2 packages (hw interface, bringup, MoveIt) | Lint, release automation | None |
| openarm_teleop | 24 | Bilateral/unilateral teleoperation | test.yaml (lint + build) | communication_test.cpp |
| openarm_isaac_lab | 62 | Isaac Lab simulation | None | None |

### Key Source Files Analyzed

- `openarm_ros2/openarm_hardware/src/v10_simple_hardware.cpp` -- ros2_control `SystemInterface` with CAN bus, lifecycle management, configurable PD gains, arm prefix for bimanual setups
- `openarm_ros2/openarm_hardware/include/openarm_hardware/v10_simple_hardware.hpp` -- Motor types (DM8009, DM4340, DM4310), CAN IDs, default gains
- `openarm_teleop/src/controller/control.hpp` -- Bilateral/unilateral controller with dynamics compensation, vibration detection
- `openarm_can/` -- C++/Python CAN library with CMake, nanobind, multi-platform CI

### Criterion Ratings

#### 1. Code Organization: 8/10

Multi-repo architecture matching hardware/software/simulation boundaries. ROS2 packages: metapackage, bimanual MoveIt config, bringup, hardware. Clean separation throughout. `.clang-format`, `.cmake-format.py`, `.pre-commit-config.yaml` across repos.

#### 2. Documentation: 9/10

Dedicated docs site at docs.openarm.dev (Docusaurus). Covers: getting-started, safety guide, hardware (assembly, BOM, specs, wiring), software (CAN, controls, ROS2, Ubuntu), simulation, teleop, FAQ, purchase. `CONTRIBUTING.md` and `CODE_OF_CONDUCT.md` everywhere.

#### 3. Build System: 7/10

openarm_can has best setup (CMake, multi-platform CI, nanobind, Fedora packaging). openarm_ros2 has lint CI and release automation. openarm_teleop has lint+build CI. Others lack CI. Pre-commit hooks consistent across repos.

#### 4. Hardware Abstraction: 10/10

Complete stack: CAN bus with Damiao motors, ros2_control SystemInterface, bimanual MoveIt2, Isaac Lab simulation, bilateral/unilateral teleoperation, URDF/XACRO, full CAD + BOM. From design files to deployed robot.

#### 5. Control Architecture: 8/10

Motor-level PD control (kp: 10-70, kd: 0.5-2.75). ros2_control lifecycle. Bilateral teleop with dynamics compensation, vibration detection (velocity windowing). MoveIt2 planning. Safety guide, configurable gains, return-to-zero.

#### 6. Community & Maintenance: 9/10

22 contributors. Same-day issue response. Dependabot. Discord. Commercial backing. Active development. Purchase pathway. CONTRIBUTING.md everywhere.

#### 7. Extensibility: 8/10

Open CAD, CAN Python bindings, ros2_control plugins, Isaac Lab, bimanual arm_prefix, pluggable teleop dynamics. Limited by single platform (V10) and motor vendor (Damiao).

#### 8. Testing: 4/10

openarm_can: build tests + Python import test. openarm_teleop: single communication test. openarm_ros2, openarm_isaac_lab, openarm_description: no tests. Significant gap for a hardware project.

#### 9. Overall Rating: 7.9/10

Most complete hardware-to-software stack. Exceptional documentation, community, hardware abstraction. Testing gap is the main concern. Commercial backing provides sustainability.

---

## 4. Open X-Humanoid (TienKung-Lab + XR-1)

**Organization:** https://github.com/Open-X-Humanoid (Beijing Humanoid Robot Innovation Center)

### 4a. TienKung-Lab

**Repository:** https://github.com/Open-X-Humanoid/TienKung-Lab
**Stars:** 617 | **Forks:** 86 | **License:** BSD-3-Clause
**Created:** July 2025 | **Last Commit:** November 27, 2025 | **Contributors:** 1

RL-based locomotion control for full-sized TienKung humanoid using IsaacLab + AMP-style rewards with periodic gait mechanics. Sim2sim (MuJoCo) and sim2real validated.

### 4b. XR-1

**Repository:** https://github.com/Open-X-Humanoid/XR-1
**Stars:** 287 | **Forks:** 40 | **License:** Other
**Created:** December 2025 | **Last Commit:** January 26, 2026 | **Contributors:** 2

Vision-Language-Action framework built on LeRobot v2.1 for multi-embodiment manipulation. Three-stage training pipeline. Supports Franka, UR-5e, TienKung 2.0, AgileX.

### Criterion Ratings

#### 1. Code Organization: 6/10

TienKung-Lab follows IsaacLab extension pattern (derivative). rsl_rl/ is vendored fork. XR-1 vendors entire LeRobot codebase. "Fork and modify" pattern rather than modular extensions.

#### 2. Documentation: 5/10

README-only docs. TienKung-Lab has GIF demos and workflow pipeline. XR-1 has training commands and deployment info. No API docs, no tutorials, no docs sites. Chinese comments in XR-1 source.

#### 3. Build System: 3/10

TienKung-Lab: legacy setup.py, no CI despite .github/ directory. XR-1: Poetry but no CI. Weakest build infrastructure surveyed.

#### 4. Hardware Abstraction: 7/10

TienKung-Lab: IsaacSim training, MuJoCo sim2sim, real deployment (separate repo), URDF/MJCF/USD. XR-1: 4-platform deployment. Thin abstraction layers.

#### 5. Control Architecture: 7/10

TienKung-Lab: AMP rewards, periodic gaits, GMR retargeting, ray-casting, CircularBuffer. XR-1: three-stage VLA, Pi0 policy, action horizon with exponential weighting. No safety mechanisms.

#### 6. Community & Maintenance: 5/10

TienKung-Lab: 1 contributor, 16 commits, last Nov 2025. XR-1: 2 contributors, 10 commits, last Jan 2026. Low activity despite organizational backing.

#### 7. Extensibility: 5/10

IsaacLab dependency creates high barrier. Vendored forks limit upstream tracking. TienKung-specific coupling.

#### 8. Testing: 1/10

Neither repository has any automated tests. No test files, no CI. Critical deficiency for real-robot deployment projects.

#### 9. Overall Rating: 4.9/10

Impressive research results (sim2real locomotion, multi-embodiment VLA) but weakest software engineering practices surveyed. Best viewed as research prototypes.

---

## 5. Comparative Summary

### Scoring Matrix

| Criterion | Weight | LearningHumanoidWalking | LocoMuJoCo | OpenArm | Open X-Humanoid |
|---|---|---|---|---|---|
| Code Organization | 15% | **9** | 8 | 8 | 6 |
| Documentation | 10% | 7 | **9** | **9** | 5 |
| Build System | 10% | **9** | 5 | 7 | 3 |
| Hardware Abstraction | 15% | 6 | 7 | **10** | 7 |
| Control Architecture | 15% | 7 | **8** | 8 | 7 |
| Community & Maintenance | 10% | 6 | 7 | **9** | 5 |
| Extensibility | 10% | 8 | **9** | 8 | 5 |
| Testing | 15% | **9** | 7 | 4 | 1 |
| **Weighted Average** | | **7.6** | **7.5** | **7.9** | **4.9** |

### Key Takeaways

1. **OpenArm leads overall** due to the most complete hardware-to-software stack, excellent documentation, and strong community. Its testing gap is the main concern.

2. **LearningHumanoidWalking excels in software engineering** -- best testing, best build system, cleanest code organization. Limited by being simulation-only with a single contributor.

3. **LocoMuJoCo is the research benchmark leader** -- widest robot support, best algorithm coverage, excellent documentation. Aging build tooling and development slowdown are concerns.

4. **Open X-Humanoid shows impressive research results** but the weakest software practices. The absence of testing and CI is a red flag for any project deploying to real robots.

### Strongest Aspects per Project

| Project | Best At |
|---|---|
| LearningHumanoidWalking | Testing (9), Build System (9), Code Organization (9) |
| LocoMuJoCo | Documentation (9), Extensibility (9), Robot Diversity (12+ humanoids) |
| OpenArm | Hardware Abstraction (10), Community (9), Documentation (9) |
| Open X-Humanoid | Sim2Real validation, VLA research |

---

## 6. Recommendations for Steveros

Based on this analysis, the following patterns are worth adopting:

### From LearningHumanoidWalking
- **Testing pattern**: Auto-discovering environments and running parametrized tests across all of them. The `conftest.py` + `test_environments.py` pattern could be adapted for Steveros controllers.
- **Abstract base class design**: The `BaseHumanoidEnv` pattern with 4 abstract methods is a clean template for extensible environments.
- **CI setup**: The dual lint+test workflow with slow/fast test separation is practical.
- **Modern Python tooling**: `uv` + `pyproject.toml` + Ruff.

### From LocoMuJoCo
- **Numbered tutorials**: Progressive tutorial structure (00-11) for onboarding.
- **Task factory pattern**: Clean separation of environment construction from algorithm logic.
- **Dual backend support**: MuJoCo/MJX pattern for CPU/GPU flexibility.

### From OpenArm
- **Multi-repo architecture**: Separate repos for hardware, firmware, ROS2, simulation -- matching team boundaries.
- **Comprehensive documentation site**: Docusaurus-based docs covering hardware assembly through software operation.
- **ROS2 hardware interface**: The `SystemInterface` implementation with configurable CAN interface, arm prefix, and PD gains is a good reference for Steveros hardware integration.
- **Bilateral teleoperation**: Force-feedback control with dynamics compensation and vibration suppression.

### From Open X-Humanoid
- **Motion retargeting**: GMR pipeline for adapting human motion capture to robot kinematics.
- **AMP rewards**: Adversarial motion prior training for natural locomotion.
- **Multi-stage VLA training**: Three-stage pipeline (representation -> pre-training -> fine-tuning) for manipulation.

### Anti-patterns to Avoid
- Vendoring entire upstream frameworks (XR-1's LeRobot fork, TienKung-Lab's RSL_RL fork)
- Deploying to real robots without automated tests (Open X-Humanoid)
- Commented-out CI steps (LocoMuJoCo's flake8)
- Hardcoded joint names in hardware interfaces (OpenArm's TODO comment)

---

*Report generated 2026-03-09 for the SteveBots/Steveros project.*
