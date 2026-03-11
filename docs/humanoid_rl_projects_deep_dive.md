# Deep Dive: Humanoid RL Locomotion & Manipulation Projects

**Date**: 2026-03-09
**Analyst**: SteveBots Research
**Scope**: Three RL-based robotics repositories analyzed at the source-code level

---

## Project 1: Berkeley Humanoid (HybridRobotics)

**Repositories analyzed**:
- `HybridRobotics/isaac_berkeley_humanoid` (273 stars, 29 forks) -- RL training code
- `HybridRobotics/Berkeley-Humanoid-Lite` (1,243 stars, 180 forks) -- Full platform (context)

**Last activity**: isaac_berkeley_humanoid: 2024-10-12 (2 commits total); Berkeley-Humanoid-Lite: 2025-09-11 (active)
**Language**: Python | **License**: BSD-3-Clause / MIT
**Contributors**: 1 (isaac_berkeley_humanoid), multiple (Berkeley-Humanoid-Lite)

### Architecture Overview

The `isaac_berkeley_humanoid` repo is an IsaacLab extension package for training locomotion policies. It follows NVIDIA's extension pattern:

```
exts/berkeley_humanoid/
  berkeley_humanoid/
    actuators/          -- Custom IdentifiedActuator (extends DCMotor with friction model)
    assets/             -- USD robot model + config.yaml
    tasks/
      locomotion/velocity/
        config/         -- Rough/flat env configs per robot
        mdp/            -- rewards.py, events.py, curriculums.py
        velocity_env_cfg.py  -- Main env configuration
    terrains/           -- Terrain generator config
scripts/
  rsl_rl/              -- train.py, play.py, cli_args.py
pyproject.toml          -- isort + pyright config
.pre-commit-config.yaml -- 7 hooks (black, flake8, isort, pyupgrade, codespell, etc.)
```

Berkeley-Humanoid-Lite extends this with sim2sim (MuJoCo), sim2real deployment, teleop, Docker, ONNX checkpoints, and YAML configs for multiple robot variants (biped, humanoid, humanoid_legs).

### Key Source Code Analysis

**Custom Actuator Model** (`actuator_pd.py`):
```python
class IdentifiedActuator(DCMotor):
    def compute(self, control_action, joint_pos, joint_vel):
        control_action = super().compute(control_action, joint_pos, joint_vel)
        # Apply friction: static tanh + dynamic viscous
        control_action.joint_efforts -= (
            self.friction_static * torch.tanh(joint_vel / self.activation_vel)
            + self.friction_dynamic * joint_vel
        )
        return control_action
```
This is a well-designed physically-motivated actuator model. The static friction uses `tanh` saturation and dynamic friction is viscous. This provides a good sim-to-real bridge for the identified actuator parameters.

**Reward Design** (`rewards.py`): Three custom biped rewards:
- `feet_air_time`: Rewards steps within [threshold_min, threshold_max] duration, zero for small commands
- `feet_air_time_positive_biped`: Enforces single-stance (exactly one foot in contact) -- critical for bipedal gait
- `feet_slide`: Penalizes foot velocity while in contact with ground

**Environment Config** (`velocity_env_cfg.py`): Clean `@configclass` hierarchy with:
- Domain randomization on 7+ parameters (friction 0.2-1.25, mass +/-10%, armature, joint defaults)
- Curriculum on terrain difficulty, push forces, and velocity commands
- Height scanner observations with noise corruption
- Termination on base contact or timeout

### Detailed Ratings

#### 1. Code Organization: 8/10

**Strengths**:
- Clean IsaacLab extension pattern with proper `__init__.py` chains and `extension.toml`
- MDP components (rewards, events, curriculums, terminations) separated into individual modules
- Configuration hierarchy using `@configclass` inheritance: base env -> robot-specific overrides
- Custom actuator cleanly extends Isaac's `DCMotor` base class
- Berkeley-Humanoid-Lite adds `source/`, `scripts/`, `configs/`, `checkpoints/` separation

**Weaknesses**:
- isaac_berkeley_humanoid is extremely minimal (26 Python files, 2 commits) -- more a config dump than a framework
- Deeply nested paths: `exts/berkeley_humanoid/berkeley_humanoid/tasks/locomotion/velocity/config/berkeley_humanoid/agents/rsl_rl_cfg.py`
- No utilities, data processing, or analysis scripts

#### 2. Documentation: 5/10

**Strengths**:
- README covers installation, training, deployment
- FAQ addresses joint torque limits, armature, and friction parameters
- Berkeley-Humanoid-Lite has a separate GitBook site
- Paper reference (arXiv:2407.21781)

**Weaknesses**:
- Terse README with no architecture diagrams, reward explanations, or hyperparameter guidance
- No inline API documentation beyond license headers
- No tutorials for reward customization or robot addition

#### 3. Build System: 6/10

**Strengths**:
- `pyproject.toml` with isort/pyright configuration
- `.pre-commit-config.yaml` with 7 hooks: black (line 120), flake8 (+simplify, +return plugins), isort, pyupgrade (py37+), codespell, trailing-whitespace, detect-private-key, debug-statements
- Berkeley-Humanoid-Lite adds Docker + `docker-compose.yaml`

**Weaknesses**:
- **Zero CI/CD**: No `.github/workflows/` in any HybridRobotics repo
- Pre-commit hooks never enforced by automation
- No dependency specification in isaac_berkeley_humanoid (relies on IsaacLab)

#### 4. Hardware Abstraction: 7/10

**Strengths**:
- `IdentifiedActuator` provides physically meaningful sim-to-real bridge with identified friction parameters
- Berkeley-Humanoid-Lite offers three deployment paths: IsaacLab, MuJoCo sim2sim, real hardware
- ONNX policy export for embedded deployment
- Multiple robot configs (biped, humanoid, humanoid_legs)

**Weaknesses**:
- isaac_berkeley_humanoid alone is simulation-only
- Friction parameters hard-coded rather than loaded from calibration files

#### 5. Control Architecture: 7/10

**Strengths**:
- Reward structure: velocity tracking (weights 1.0/0.5), penalty terms for torque, smoothness, slip, contacts
- Biped-specific `feet_air_time_positive_biped` enforces single-stance gait
- Curriculum learning on terrain, push disturbances, velocity
- Domain randomization on friction, mass, armature, joint positions, friction models

**Weaknesses**:
- No safety layer beyond joint torque caps
- Single control mode (PD position)
- No impedance or hybrid control

#### 6. Community & Maintenance: 3/10

- isaac_berkeley_humanoid: 2 commits, 0 PRs, 3 open issues (2 unanswered), 1 contributor
- Berkeley-Humanoid-Lite more active (1,243 stars, pushed 2025-09-11)
- No CONTRIBUTING.md or contribution guidelines

#### 7. Extensibility: 6/10

- IsaacLab extension pattern enables new task/robot registration
- `@configclass` inheritance for parameter overrides
- Modular MDP components
- Tightly coupled to IsaacLab 1.0.0 / IsaacSim 4.0.0

#### 8. Testing: 1/10

- **Zero test files** in the entire repository
- No unit tests, integration tests, or simulation validation
- No CI to run tests

---

## Project 2: Walk These Ways (Improbable-AI / CMU)

**Repository**: `Improbable-AI/walk-these-ways`
**Stars**: 1,292 | **Forks**: 213 | **Open Issues**: 9
**Last activity**: 2024-06-16
**Language**: Python | **License**: Based on legged_gym (ETH RSL)
**Contributors**: 4 (22 commits from primary author gmargo11)

### Architecture Overview

Walk These Ways implements "Multiplicity of Behavior" (MoB) -- a single RL policy that produces diverse gaits on the Unitree Go1 quadruped. Three-package structure:

```
go1_gym/              -- Simulation environment (65 Python files)
  envs/
    base/             -- BaseTask, LeggedRobot (~800 lines), curriculum, config
    go1/              -- Go1 config, VelocityTrackingEasyEnv
    rewards/          -- CoRLRewards (24 reward functions)
    wrappers/         -- HistoryWrapper (30-step observation stacking)
  utils/              -- math_utils, terrain generation

go1_gym_learn/        -- RL training
  ppo/                -- Standard PPO (actor_critic, rollout_storage)
  ppo_cse/            -- PPO with Conditioning for Structured Exploration
  eval_metrics/       -- Domain randomization evaluation

go1_gym_deploy/       -- Real robot deployment
  docker/             -- Dockerfile (L4T Jetson), Makefile, entrypoint
  envs/               -- LCMAgent (real robot interface)
  lcm_types/          -- LCM schema (.lcm) + generated Python
  scripts/            -- deploy_policy.py
  utils/              -- DeploymentRunner, state estimator, command profiles
  installer/          -- install_deployment_code.sh
  autostart/          -- systemd-style robot startup
  tests/              -- check_camera_msgs.py (LCM validation)
```

Also includes: pretrained checkpoints (`.jit` models), Go1 + Mini Cheetah URDF/meshes, actuator network (`unitree_go1.pt`), terrain textures.

### Key Source Code Analysis

**LeggedRobot** (`legged_robot.py`) -- monolithic but comprehensive:
- `step()`: Action clipping -> control decimation (multiple physics substeps) -> torque computation
- `_compute_torques()`: PD controller or neural actuator network, with hip flexion reduction and motor strength scaling
- `compute_observations()`: Configurable 70-dim observation (gravity, joints, commands, clock, prev actions, optional privileged)
- Domain randomization: friction, restitution, COM, mass, motor strength/offset, gravity, Kp/Kd, lag timesteps
- Gait curriculum: per-category (trot/pace/bound/pronk) Lipschitz curricula with distributional command sampling
- `_step_contact_targets()`: Sine-wave based desired contact timing for gait pattern enforcement

**Actor-Critic Architecture** (`ppo_cse/actor_critic.py`):
- Teacher-student design:
  - **Environment Factor Encoder**: Privileged observations -> latent [256, 128]
  - **Adaptation Module**: Observation history -> latent [256, 32] (student replaces encoder at deployment)
  - **Actor**: [512, 256, 128] MLP with ELU
  - **Critic**: [512, 256, 128] MLP
- `act_teacher()` uses privileged info; `act_inference()` uses adaptation module

**DeploymentRunner** (`deployment_runner.py`):
- Calibration sequence with button-activated stages (R2 to calibrate, R2 to start)
- Main control loop with emergency orientation check (|roll| or |pitch| > 1.6 rad -> recalibrate in low pose)
- Button-triggered logging with pickle serialization
- Multi-agent support (can run multiple control agents simultaneously)

**CoRLRewards** (`corl_rewards.py`) -- 24 reward functions including:
- Velocity tracking (linear + angular)
- Contact-shaped rewards (force-based + velocity-based gait tracking)
- Action smoothness (two variants: 1st and 2nd order)
- Raibert heuristic (footstep placement relative to velocity)
- Orientation control (roll/pitch commands)
- Hop symmetry, jump height tracking

**Training Config** (`train.py`) -- 150+ lines of `Cfg.x.y = z`:
- 15-dimensional command space: lin_vel_{x,y}, ang_vel_yaw, body_height, gait_{frequency,phase,offset,bound,duration}, footswing_height, body_{pitch,roll}, stance_{width,length}
- Curriculum: tracking thresholds (0.7/0.8/0.9) with gaitwise distributional commands
- Domain randomization: friction [0.1,3.0], restitution [0,0.4], mass [-1,3]kg, gravity [-1,1]m/s^2, motor strength [0.9,1.1]
- Actuator network control (`control_type = "actuator_net"`)

### Detailed Ratings

#### 1. Code Organization: 7/10

**Strengths**:
- Clean three-way split: simulation, training, deployment
- LCM message types in both `.lcm` schema and `.py` generated forms
- 24 reward functions collected in single `CoRLRewards` class
- Deployment mirrors sim structure (same observation construction, action scaling)
- Pretrained checkpoints included

**Weaknesses**:
- Monolithic `legged_robot.py` (~800+ lines) with everything in one class
- Heavy code duplication: `ppo/` and `ppo_cse/` duplicate actor_critic, ppo, rollout_storage with minor changes
- Configuration by global `Cfg` mutation (150+ `Cfg.x.y = z` lines) rather than config files
- Binary assets (meshes, textures, model weights) committed directly to git

#### 2. Documentation: 6/10

**Strengths**:
- Full sim-to-real pipeline in README: installation, training, deployment, safety
- Safety section: PowerProtect level 9, physical precautions
- RC controller mapping diagram
- Paper reference (CoRL 2022)

**Weaknesses**:
- No API docs for 24 reward functions
- No explanation of 15D command space beyond paper
- No architecture diagrams
- No troubleshooting guide

#### 3. Build System: 4/10

**Strengths**:
- `setup.py` for `go1_gym`, separate `setup.py` for deploy
- Dockerfile for Jetson (L4T), Makefile for Docker management
- Installer scripts for Go1 deployment

**Weaknesses**:
- **Zero CI/CD** (no `.github/workflows/`)
- No `pyproject.toml`, no linting, no pre-commit hooks
- Dependencies only in README (PyTorch 1.10, CUDA 11.3)
- Docker image pinned to `nvcr.io/nvidia/l4t-pytorch:r32.6.1` -- very specific

#### 4. Hardware Abstraction: 8/10

**Strengths**:
- **Complete sim-to-real pipeline** -- the strongest aspect of this project
- `LCMAgent` abstracts real robot communication over LCM protocol
- `DeploymentRunner`: calibration, control loop, emergency stop, logging
- Actuator network (`unitree_go1.pt`) models real motor dynamics
- Emergency stop on orientation (roll/pitch > 1.6 rad)
- Docker-based deployment isolation on robot hardware
- RC controller integration for real-time commands
- State estimator abstraction

**Weaknesses**:
- Only Unitree Go1 -- no abstraction for other robots
- Hard-coded network config (192.168.123.15)
- No sim-to-real transfer validation metrics

#### 5. Control Architecture: 8/10

**Strengths**:
- 15D command space enables diverse gaits (velocity, height, frequency, phase, offset, bound, duration, footswing, pitch, roll, stance)
- Lipschitz-based curriculum with per-gait-category distributions
- Teacher-student architecture: privileged encoder + adaptation module from history
- 24 reward functions across velocity, gait, smoothness, collision domains
- Actuator network for realistic motor dynamics
- 30-step history wrapper for adaptation
- Control decimation for realistic timing
- Domain randomization: friction, mass, COM, motor strength/offset, gravity, lag, Kp/Kd

**Weaknesses**:
- PD + actuator net only -- no impedance/force control
- Single safety mechanism (orientation check)
- Calibration uses fixed `time.sleep(0.05)` -- not compute-robust

#### 6. Community & Maintenance: 5/10

**Strengths**:
- 1,292 stars, 213 forks
- Some issues answered (3 with multiple comments)
- External PRs accepted

**Weaknesses**:
- Last commit 2024-06-16 -- 1.5+ years inactive
- 6 of 9 open issues unanswered
- Bus factor of 1 (gmargo11: 22 of 25 commits)
- No releases or versioning

#### 7. Extensibility: 5/10

**Strengths**:
- Method-based rewards easy to extend
- Configurable command space
- Go1 + Mini Cheetah URDFs included

**Weaknesses**:
- Global mutable `Cfg` is brittle
- No plugin or robot registration system
- Code duplication between PPO variants

#### 8. Testing: 2/10

- `scripts/test.py`: Environment smoke test (1000 zero-action steps) -- no assertions
- `go1_gym_deploy/tests/check_camera_msgs.py`: LCM camera validation tool
- Neither uses a test framework (pytest/unittest)
- No reward unit tests, no policy validation, no regression tests

---

## Project 3: Bi-DexHands (PKU-MARL)

**Repository**: `PKU-MARL/DexterousHands`
**Stars**: 982 | **Forks**: 116 | **Open Issues**: 36
**Last activity**: 2025-02-18 (bug fix)
**Language**: Python | **License**: Apache 2.0
**Contributors**: 7 (107 from cypypccpy, 49 from Shengjie-bob, 37 from PKU-YYang)
**Python files**: 140

### Architecture Overview

Bi-DexHands is a benchmark platform for bimanual dexterous manipulation with Shadow Hand robots in Isaac Gym.

```
bidexhands/
  algorithms/
    rl/               -- PPO, TRPO, SAC, TD3, DDPG (5 single-agent algorithms)
    marl/             -- HAPPO, HATRPO, MAPPO, IPPO, MADDPG (5 multi-agent)
    mtrl/             -- MTPPO, MTSAC, MTTRPO (3 multi-task)
    metarl/           -- MAML/ProMP (2 meta-RL)
    offrl/            -- BCQ, IQL, TD3-BC + PPO-collect (4 offline)
    utils/            -- distributions, CNN, activations
  tasks/
    hand_base/        -- BaseTask, VecTask, MultiVecTask, MetaVecTask (5 base classes)
    shadow_hand_*.py  -- 16+ task environments (500-1000+ lines each)
    shadow_hand_meta/ -- Meta-learning task variants
  cfg/                -- YAML configs per algorithm (15+ config files)
assets/
  mjcf/               -- Object models: bottle_cap, door, cup, bucket, scissors, pen, etc.
  urdf/               -- Shadow Hand description + YCB objects
dataset/              -- Benchmark CSVs (10 seeds x algo x task)
setup.py              -- bidexhands v0.1.2
```

### Key Source Code Analysis

**BaseTask** (`hand_base/base_task.py`):
- Manages Isaac Gym lifecycle: sim creation, viewer, physics stepping
- Domain randomization via `apply_randomizations()`: handles sim_params, actor_params, observations, actions
- Supports Gaussian and uniform noise with scheduled scaling
- Actor parameter randomization: friction, restitution, mass, scale, COM, color
- Buffer allocation: obs_buf, rew_buf, reset_buf, progress_buf

**ShadowHandOver** (`shadow_hand_over.py`):
- 398-dimensional observation space per environment:
  - Right hand: 24 DOF pos + 24 vel + 24 forces + 65 fingertip states + 30 fingertip forces + 3 base pos + 3 base rot + 20 actions
  - Left hand: mirror of right hand observations
  - Object: 7 pose + 3 lin_vel + 3 ang_vel + 7 goal pose + 5 quat diff
- Optional point cloud: +2,304 dims (768 points x 3 coords from depth camera)
- Reward: `exp(-0.2 * (pos_distance * scale + rot_distance))` + action penalty + success/fall bonuses
- Success: object within 0.03m of goal
- Supports 7+ object types: block, egg, pen, YCB banana/can/mug/brick
- Force-torque sensors on all 10 fingertips
- Tendon simulation with stiffness 30, damping 0.1

**HAPPO Trainer** (`marl/happo_trainer.py`):
- Clean implementation of Heterogeneous-Agent PPO
- PPO update with importance-weighted clipped objective
- Factor-based policy loss: `factor_batch * min(surr1, surr2)` (heterogeneous agent advantage)
- Support for: PopArt value normalization, Huber loss, clipped value loss, recurrent policies, active masks
- Gradient clipping via `max_grad_norm`
- Advantage normalization with mean/std

**Algorithm structure** (consistent across all 19 algorithms):
```
algorithm_name/
  __init__.py
  module.py     -- Neural network definitions (Actor, Critic)
  storage.py    -- Replay/rollout buffer
  algorithm.py  -- Training logic (PPO update, SAC update, etc.)
```

### Detailed Ratings

#### 1. Code Organization: 7/10

**Strengths**:
- Excellent algorithm taxonomy: `rl/`, `marl/`, `mtrl/`, `metarl/`, `offrl/` with 19 algorithms
- Consistent internal pattern per algorithm: module.py, storage.py, trainer.py
- Task hierarchy: BaseTask -> VecTask -> MultiVecTask -> specific tasks
- 140 Python files with clear naming
- YAML config files per algorithm
- Clean `pip install -e .` via `setup.py`
- Rich object asset library from SAPIEN and YCB

**Weaknesses**:
- Massive code duplication across `shadow_hand_*.py` files (each 500-1000+ lines, ~80% shared)
- Algorithm implementations also heavily duplicated (HAPPO/HATRPO/MAPPO share ~80%)
- No shared base class for task-specific environments beyond generic BaseTask
- Asset files bloat repository (hundreds of mesh/texture files in git)

#### 2. Documentation: 7/10

**Strengths**:
- Comprehensive README: task descriptions, GIF demos, performance tables
- Python API example: `bidexhands.make('ShadowHandOver', 'ppo')`
- Clear algorithm list with paper references
- Training/testing commands for every algorithm category
- Point cloud observation documentation
- Benchmark results with 10-seed CSVs
- NeurIPS 2022 paper reference

**Weaknesses**:
- No API reference documentation
- No tutorial for adding new tasks or objects
- No architecture diagram showing class hierarchy
- Config parameters undocumented (must read YAML/source)

#### 3. Build System: 3/10

**Strengths**:
- `setup.py` with dependencies (gym, matplotlib 3.5.1, numpy 1.23.0, rl-games 1.5.2)
- `pip install -e .` workflow

**Weaknesses**:
- **Zero CI/CD** (no `.github/workflows/`)
- No linting, formatting, or pre-commit hooks
- No `pyproject.toml` -- legacy `setup.py` only
- Pinned to old versions (numpy 1.23.0, Python 3.7/3.8)
- No Docker support
- Isaac Gym Preview dependency (closed-source, specific version)

#### 4. Hardware Abstraction: 3/10

**Strengths**:
- PhysX GPU-accelerated simulation
- Point cloud observation bridges toward real perception
- Force-torque sensors, depth cameras

**Weaknesses**:
- **Simulation-only** -- no real robot deployment code
- No sim-to-real pipeline
- No real Shadow Hand interface
- Hard-coded to Shadow Hand morphology
- No sim-to-real gap discussion

#### 5. Control Architecture: 6/10

**Strengths**:
- Relative and absolute joint control modes
- Tendon simulation with configurable stiffness/damping
- Force-torque on 10 fingertips
- JIT-compiled reward computation
- 398-dim observation captures rich state
- Multi-agent formulation for heterogeneous control
- Domain randomization on object/physics parameters

**Weaknesses**:
- No safety mechanisms
- No impedance/compliant control
- Simple exponential distance reward -- limited shaping
- No contact-aware control or slip detection
- No real-time considerations

#### 6. Community & Maintenance: 4/10

**Strengths**:
- 7 contributors (most diverse team)
- 982 stars, 116 forks
- Bug fixes still merged (2025-02-18)
- Academic benchmark drives usage

**Weaknesses**:
- 36 open issues, most unanswered
- Known bugs unresolved for 2+ years (compute_observations correctness, 2023-12)
- No CONTRIBUTING.md
- 3 total PRs (all from maintainers)
- Primary author: 76% of commits

#### 7. Extensibility: 6/10

**Strengths**:
- Clear algorithm addition pattern (module/storage/trainer)
- YAML config per algorithm
- Task templates from existing `shadow_hand_*.py`
- Multi-task and meta-learning infrastructure built in
- Python API (`bidexhands.make()`)

**Weaknesses**:
- New hand models require deep URDF/joint modifications
- No registration system for tasks
- Code duplication means changes must be replicated
- Tightly coupled to Isaac Gym Preview

#### 8. Testing: 2/10

**Strengths**:
- Benchmark CSVs (`dataset/`) with 10-seed results across algorithms and tasks

**Weaknesses**:
- **Zero test files** -- no unit tests, integration tests, or automated validation
- CSVs are reference data, not automated tests
- No reward verification, observation validation, or regression detection

---

## Comparative Summary

| Criterion                   | Berkeley Humanoid | Walk These Ways | Bi-DexHands |
|-----------------------------|:-----------------:|:---------------:|:-----------:|
| 1. Code Organization        |        8          |        7        |      7      |
| 2. Documentation             |        5          |        6        |      7      |
| 3. Build System              |        6          |        4        |      3      |
| 4. Hardware Abstraction      |        7          |        8        |      3      |
| 5. Control Architecture      |        7          |        8        |      6      |
| 6. Community & Maintenance   |        3          |        5        |      4      |
| 7. Extensibility             |        6          |        5        |      6      |
| 8. Testing                   |        1          |        2        |      2      |
| **9. Overall Rating**        |      **5.4**      |      **5.6**    |    **4.8**  |

*Overall = weighted average: Code Org 15%, Docs 10%, Build 10%, Hardware 15%, Control 15%, Community 10%, Extensibility 10%, Testing 15%*

---

## Key Findings

### Common Weaknesses Across All Three

1. **Zero CI/CD everywhere**: None of the three repos have GitHub Actions, CircleCI, or any automated pipeline. This is systemic in robotics RL research code.

2. **Negligible testing**: Combined across 231 Python files, there are effectively 0 proper test files with assertions. The only "tests" are visualization demos and LCM message inspectors.

3. **Single-maintainer risk**: All three dominated by 1-2 contributors. Bus factor is effectively 1.

4. **Paper-driven development**: These are code releases accompanying papers, not production software. Quality correlates with "enough to reproduce results" rather than "maintainable over time."

5. **Isaac Gym lock-in**: All three depend on NVIDIA's closed-source Isaac Gym (or IsaacLab/IsaacSim), creating fragile version-pinned dependency chains.

6. **Configuration anti-patterns**: Walk These Ways uses 150+ lines of global mutation; DexterousHands uses YAML but doesn't validate; Berkeley uses `@configclass` (best of the three).

### Notable Strengths By Project

| Project | Standout Feature | Evidence |
|---------|-----------------|----------|
| Berkeley Humanoid | Best code quality | Pre-commit hooks (7 tools), `@configclass` hierarchy, clean actuator model with identified friction |
| Walk These Ways | Best sim-to-real | Complete deployment pipeline: Docker/Jetson, LCM protocol, actuator network, calibration, emergency stop, RC controller, logging |
| Bi-DexHands | Broadest algorithm coverage | 19 algorithms across 5 RL paradigms (single, multi-agent, multi-task, meta, offline), 16+ tasks, 10-seed benchmarks |

### Recommendations for SteveBots

1. **Walk These Ways' deployment architecture** is the most relevant reference for sim-to-real transfer. The `DeploymentRunner` pattern (calibration -> control loop -> emergency stop -> logging) is directly applicable to humanoid arm control.

2. **Berkeley Humanoid's IsaacLab extension pattern** is the cleanest for organizing RL training environments. Their `@configclass` approach and custom `IdentifiedActuator` with friction identification are worth adopting.

3. **Bi-DexHands' algorithm diversity** is valuable for benchmarking different RL approaches on manipulation tasks, but the code quality is lowest and lacks any real-robot pipeline.

4. All three projects demonstrate that **domain randomization** (friction, mass, motor parameters, gravity, lag) is essential for sim-to-real transfer.

5. **Competitive advantage opportunity**: All three projects lack CI/CD, tests, and multi-contributor workflows. Steveros already has better software practices than these high-profile RL research repos. Investing in testing and CI would put the project in the top tier of robotics open-source.

---

## Repository Links

| Project | URL | Stars | Last Active |
|---------|-----|-------|-------------|
| Berkeley Humanoid (training) | https://github.com/HybridRobotics/isaac_berkeley_humanoid | 273 | 2024-10-12 |
| Berkeley Humanoid Lite (full) | https://github.com/HybridRobotics/Berkeley-Humanoid-Lite | 1,243 | 2025-09-11 |
| Walk These Ways | https://github.com/Improbable-AI/walk-these-ways | 1,292 | 2024-06-16 |
| Bi-DexHands | https://github.com/PKU-MARL/DexterousHands | 982 | 2025-02-18 |

*Report generated March 2026 for SteveBots/Steveros.*
