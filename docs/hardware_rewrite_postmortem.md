# SteveROS Hardware Interface Rewrite — Complete Record

**Date**: 2026-02-24
**Author**: Human + Claude (research & implementation agents)
**Status**: Implementation complete, pending hardware testing

---

## 1. The Problem

During single motor testing of motor 21 (right shoulder pitch, Robstride RS03) via `ros2_control`, the motor produced a loud noise on startup and slammed toward position zero without any user command. After ~47 seconds, a feedback timeout triggered and the controller manager deactivated the hardware.

### Root Cause

The `on_activate()` implementation in the original `steveros_hardware.cpp` had a snap-to-zero bug:

1. All state vectors initialized to `0.0` (line 154: `hw_positions_.resize(num_joints, 0.0)`)
2. States re-zeroed in `on_configure()` (lines 219-226)
3. Motor enable sent (Type 3 CAN frames) for all 20 motors
4. Only 10ms wait for feedback (`usleep(10000)` at line 247)
5. Non-blocking `recv()` drain — captured only 3-4 of 20 motors' feedback
6. Remaining motors' positions stayed at `0.0`
7. Commands set to `hw_positions_[i]` which was still `0.0`
8. Kp ramp drove motors toward position 0.0 from their actual positions

Additionally, the protocol layer used hardcoded encoding ranges (`kTorqueMin=-12`, `kKpMax=500`, `kKdMax=5`) that were wrong for RS03 (torque ±60Nm, Kp 0-5000, Kd 0-100) and RS04 (torque ±120Nm) motors. The torque limit was also hardcoded to 12Nm at line 94, capping RS03/RS04 motors far below their actual capability.

### Log Evidence

```
[INFO]  Activated: all motors enabled (Kp ramp over 1.0s).
        ... 47 seconds of commanding position 0.0 ...
[ERROR] Feedback timeout on motor 21 ('dof_right_shoulder_pitch_03'): 0.500s since last feedback
[ERROR] Deactivating following hardware components as their read cycle resulted in an error
```

---

## 2. Approach: Two-Phase Agent Workflow

Rather than attempting a direct fix, the rewrite was split into two distinct phases with separate Claude Code agents, each receiving a purpose-built prompt.

### Why Two Phases

A single prompt asking Claude to "research and implement" produces poor results because:

- The agent re-researches mid-implementation, burning context window
- Architectural decisions get revisited after code is partially written
- Research artifacts (GitHub source, docs) pollute the context needed for writing code
- The agent second-guesses itself when it encounters implementation friction

Separating research from implementation lets each agent focus entirely on its task with a clean context window.

---

## 3. Phase 1: Research Agent

### Prompt Design

The research prompt used XML-tagged sections following Anthropic's recommended structure:

```
<role>         — Senior robotics architect, research and planning ONLY
<success_criteria> — 5 specific deliverables with measurable completion
<project_context>  — Problem statement, file inventory, what works/what's broken
<reference_implementations> — Named repos to investigate
<instructions>     — Three sequential phases (research → decisions → plan)
<constraints>      — No coding, confidence tracking, save to file
<output_format>    — Named sections with ADR format
```

**Key techniques applied:**

| Technique | Purpose |
|---|---|
| Documents before instructions | Anthropic docs show 30%+ quality improvement for multi-document tasks |
| Sequential phases with gates | "Do not start Phase 2 until Phase 1 is complete" prevents premature planning |
| Parallel subagent delegation | Three independent research streams (protocol, ros2_control, CAN patterns) run concurrently |
| Confidence tracking (HIGH/MEDIUM/LOW) | Prevents speculation from being presented as fact |
| State persistence to disk | `docs/hardware_rewrite_research.md` survives context compaction |
| Anti-overthinking guardrails | "If two approaches seem equivalent, pick the simpler one and move on" |
| Explicit success criteria | Agent knows exactly when the task is done |

### Prompt (Saved)

Full prompt saved to: `docs/hardware_rewrite_prompt_research.xml` (not saved during session — see Section 7 for reconstruction)

### Research Output

The research agent produced `docs/hardware_rewrite_research.md` (820 lines) containing:

**Phase 1 findings:**
- 11 open-source projects using Robstride RS-series motors identified and ranked
- Motor activation sequences compared across all 11 projects
- CAN driver architectures compared (threading, retries, feedback caching)
- Per-model motor parameter tables cross-verified across 3 independent sources
- ros2_control lifecycle patterns analyzed across 6 real-hardware implementations
- CAN bus timing analysis for 20 motors at 1 Mbps (59% utilization, 100Hz feasible)

**Phase 2 decisions (6 Architecture Decision Records):**

| ADR | Decision | Confidence |
|---|---|---|
| ADR-1: Threading | Background receive thread with `poll()`, 10ms timeout | HIGH |
| ADR-2: Activation | 4-phase sequence: clear faults → soft enable (Kp=0) → read positions → position hold | HIGH |
| ADR-3: State init | NAN-initialized states (5/6 reference impls agree) | HIGH |
| ADR-4: Layer separation | Three layers: protocol (header-only) → CAN driver (thread/retry) → hardware interface (lifecycle) | HIGH |
| ADR-5: Per-model params | Constexpr lookup table in protocol header, `motor_type` URDF param per joint | HIGH |
| ADR-6: Send retries | 10 retries, linear backoff (500us base), ENOBUFS/EAGAIN/EWOULDBLOCK | HIGH |

**Phase 3 rewrite plan:**
- File-by-file inventory with estimated line counts
- 4-step build order with compile verification gates
- Smoke test plan (1 motor → 3 motors → 20 motors)
- Explicit exclusion list (no gravity comp, no trajectory smoothing, no debug publishers)
- 8 ranked open questions requiring hardware testing

### Cross-Reference Agreement Matrix

The research identified where references agreed (high confidence) vs disagreed:

| Topic | Agreeing Sources | Confidence |
|---|---|---|
| States → NAN | ros2_control, ros_odrive, Reachy2, Orbita3D, gz_ros2_control | HIGH |
| Fault clear before enable | EDULITE_A3, Official protocol doc | HIGH |
| Background receive thread | EDULITE_A3, shdmsrkd | HIGH |
| Separate send/feedback mutex | EDULITE_A3, K-Scale actuator | HIGH |
| Set run mode before enable | EDULITE_A3 only | MEDIUM |

---

## 4. Phase 2: Implementation Agent

### Prompt Design

The implementation prompt was fundamentally different from the research prompt:

```
<role>         — Implementing a rewrite, all decisions are settled
<authority>    — Research doc is authoritative, do not second-guess
<files_to_read> — Exact file paths to read before writing
<build_order>  — 4 numbered steps with compile gates
<code_standards> — Style, namespaces, line targets
<constraints>  — No over-engineering, exclusion list, no feature creep
<escape_hatch> — May fetch ONE file from ONE repo if stuck on a detail
```

**Key differences from research prompt:**

| Research Prompt | Implementation Prompt |
|---|---|
| "Investigate and decide" | "The decision is made, implement it" |
| Open-ended exploration | Exact build order with file names |
| Confidence levels | `<authority>` tag — no re-evaluation |
| Success = producing a plan | Success = compiling code |
| Parallel subagents for research | "Read these files before writing" |

**The critical addition was the `<authority>` tag.** Without it, Claude naturally tries to validate every claim in the research doc, burning context and potentially introducing contradictions. With it, the agent treats the research as a spec and builds to it.

### Prompt (Saved)

Full prompt saved to: `docs/hardware_rewrite_prompt.xml`

### Session Management

The implementation agent was run in a **fresh session** (not a compacted continuation of the research session). This was deliberate:

- **Compacting** preserves a summary of prior discussion, including meta-discussion about prompt design, doubts about the research, etc. — exactly the context that causes an agent to second-guess itself.
- **Fresh session** means the agent sees only the implementation prompt and the files it reads from disk. Zero baggage.

---

## 5. Implementation Results

### What Was Built

**Step 1: Extended `robstride_protocol.hpp`** (174 → 269 lines, +95)

| Addition | Lines | Purpose |
|---|---|---|
| `MotorType` enum | 1 | RS01, RS02, RS03, RS04 |
| `MotorParams` struct | 8 | Per-model position/velocity/torque/Kp/Kd ranges |
| `get_motor_params()` | 15 | Constexpr lookup table |
| `parse_motor_type()` | 8 | String-to-enum for URDF parsing |
| `encode_stop_clear_fault()` | 13 | Type 4 frame with data[0]=1 |
| `encode_param_write()` | 24 | Type 18 frame, LE index + LE float |
| `encode_mit_command(cmd, params)` | 27 | Per-model encoding ranges overload |
| Renamed feedback constants | — | `kFbPosMin` etc. to distinguish from command ranges |

Backward compatibility maintained via legacy `encode_mit_command(cmd)` overload defaulting to RS02 ranges.

**Step 2: Created `RobstrideCanDriver`** (63 + 227 = 290 lines)

| Component | Lines | Pattern |
|---|---|---|
| `open()` | 60 | Socket create, bind, filter, SO_RCVTIMEO, thread start |
| `close()` | 20 | Stop thread, join, close socket, clear cache |
| `receive_loop()` | 28 | `poll()` 10ms timeout, decode, mutex-protected cache update |
| `send_frame()` | 25 | 10 retries, 500us linear backoff, ENOBUFS/EAGAIN/EWOULDBLOCK |
| `get_feedback()` / `has_valid_feedback()` | 15 | Mutex-protected cache reads |
| Motor control methods | 20 | `clear_fault`, `set_run_mode`, `enable_motor`, `disable_motor` |

**Step 3: Rewrote `SteveROSHardware`** (105 + 446 = 551 lines)

| Change | Detail |
|---|---|
| `JointState` struct | NAN-initialized position/velocity/effort, 0.0-initialized command |
| `JointConfig` | Added `motor_type` field |
| `on_init()` | Parses `motor_type` from URDF, uses per-model torque limits |
| `on_configure()` | Creates `RobstrideCanDriver`, calls `open()` |
| `on_activate()` | 4-phase activation (ADR-2): fault clear → soft enable → read positions → position hold |
| `on_deactivate()` | Zero-torque + disable via driver API |
| `on_cleanup()` | Driver close + reset |
| `read()` | Copies from background thread feedback cache, checks timeout |
| `write()` | Per-model `MotorParams` encoding, Kp ramp preserved |

**Step 4: Updated build and config files**

- `CMakeLists.txt`: Added `src/robstride_can_driver.cpp` to sources
- `steveros_ros2_control.xacro`: Added `<param name="motor_type">` to all 20 joints with correct types per K-Scale KBot mapping

### Final Line Counts

| File | Before | After |
|---|---|---|
| `robstride_protocol.hpp` | 174 | 269 |
| `robstride_can_driver.hpp` | — | 63 |
| `robstride_can_driver.cpp` | — | 227 |
| `steveros_hardware.hpp` | 130 | 105 |
| `steveros_hardware.cpp` | 480 | 446 |
| `CMakeLists.txt` | 53 | 53 |
| **Total** | **837** | **1163** |

The increase (+326 lines) is the CAN driver layer (290 lines) that previously didn't exist, plus per-model parameter tables (95 lines added to protocol header). The hardware interface itself (`.hpp` + `.cpp`) actually shrank from 610 → 551 lines by delegating CAN I/O to the driver.

---

## 6. Verification

### Spec Compliance Audit

Every requirement from the research doc was verified against the implementation:

| Requirement | File:Line | Verdict |
|---|---|---|
| NAN-initialized states | `steveros_hardware.hpp:39-41` | PASS |
| Per-model MotorParams (RS01-RS04) | `robstride_protocol.hpp:60-74` | PASS |
| Background receive thread (poll) | `robstride_can_driver.cpp:122-150` | PASS |
| Send retries (10, linear backoff) | `robstride_can_driver.cpp:156-181` | PASS |
| Separate send/feedback mutexes | `robstride_can_driver.hpp:56-57` | PASS |
| 4-phase activation (ADR-2) | `steveros_hardware.cpp:190-253` | PASS |
| Commands synced to actual position | `steveros_hardware.cpp:235` | PASS |
| Type 18 param write (LE layout) | `robstride_protocol.hpp:212-235` | PASS |
| Fault clear (Type 4 data[0]=1) | `robstride_protocol.hpp:196-208` | PASS |
| Per-model torque limits | `steveros_hardware.cpp:91-99` | PASS |
| motor_type URDF param | `steveros_ros2_control.xacro:21` | PASS |
| Kp ramp preserved | `steveros_hardware.cpp:400-407` | PASS |
| Feedback timeout preserved | `steveros_hardware.cpp:378-388` | PASS |
| Coordinate transforms preserved | `steveros_hardware.cpp:431-438` | PASS |
| No over-engineering | — | PASS |
| Exclusion list honored | — | PASS |
| Legacy backward compatibility | `robstride_protocol.hpp:161-165` | PASS |

### Build Verification

All 4 steps compiled successfully via `colcon build --packages-select steveros_hardware`.

### Known Minor Items

1. **Double mutex lock in `read()`**: `has_valid_feedback()` + `get_feedback()` acquires the feedback mutex twice per motor per cycle (40 locks at 100Hz for 20 motors). Not a performance issue at this scale but could be combined into a single `std::optional<MotorFeedback> try_get_feedback()` if needed.

2. **Feedback decode mask**: `fb.motor_id = (arb_id >> 8) & 0xFFFF` uses a 16-bit mask, but motor IDs are 8-bit (0-255). Upper byte is always 0 for real motor IDs so this is functionally correct but cosmetically imprecise.

---

## 7. Files Produced

| File | Purpose |
|---|---|
| `docs/hardware_interface_analysis.md` | Original root cause analysis and reference comparison (pre-existing) |
| `docs/hardware_rewrite_research.md` | Research findings, ADRs, and rewrite plan (research agent output) |
| `docs/hardware_rewrite_prompt.xml` | Implementation prompt (XML-formatted) |
| `docs/hardware_rewrite_postmortem.md` | This document |
| `steveros_hardware/include/steveros_hardware/robstride_protocol.hpp` | Extended protocol layer |
| `steveros_hardware/include/steveros_hardware/robstride_can_driver.hpp` | New CAN driver header |
| `steveros_hardware/src/robstride_can_driver.cpp` | New CAN driver implementation |
| `steveros_hardware/include/steveros_hardware/steveros_hardware.hpp` | Rewritten hardware interface header |
| `steveros_hardware/src/steveros_hardware.cpp` | Rewritten hardware interface implementation |
| `steveros_hardware/CMakeLists.txt` | Updated build file |
| `steveros_description/urdf/steveros_ros2_control.xacro` | Updated URDF with motor_type params |

---

## 8. Next Steps: Hardware Testing

Follow the smoke test plan from `docs/hardware_rewrite_research.md` §3.3:

**Test 0 — Protocol layer** (no hardware): Verify MotorParams lookup and frame encoding are correct for all 4 motor types.

**Test 1 — CAN driver** (1 motor on bench): Standalone test of the driver — clear fault, enable, soft-enable (Kp=0), read feedback, position hold, disable.

**Test 2 — Hardware interface** (1 motor via ros2_control): Launch with 1 joint. Verify motor does NOT snap to zero. Verify position feedback is plausible. Send a small position command. Verify smooth motion.

**Test 3 — Multi-motor** (3, then 20): Repeat Test 2 at scale. Monitor CAN bus utilization, feedback timeout rate, send retry rate.

### Open Questions Requiring Hardware Testing

| # | Question | Impact |
|---|---|---|
| 1 | Can 50ms inter-motor fault-clear delay be reduced? | Startup time: 1s vs 4s |
| 2 | Do RS03/RS04 need different Kd during soft-enable? | Kd=4.0 may be wrong for 0-100 range |
| 3 | Is Type 18 setRunMode needed before every enable? | Could save 600ms on startup |
| 4 | What is minimum time between enable and first feedback? | Determines Phase 2 wait time |
| 5 | Does motor mode == 2 (Run) reliably appear after enable? | Health check opportunity |
| 6 | What txqueuelen prevents ENOBUFS? | System config requirement |

---

## 9. Lessons Learned: Agent Prompt Engineering

### What Worked

1. **Separating research from implementation** was the single most impactful decision. The research agent explored freely without implementation pressure; the implementation agent built to a spec without research distraction.

2. **The `<authority>` tag** prevented the implementation agent from second-guessing the research. Without it, agents naturally re-evaluate prior decisions when they encounter friction.

3. **XML-tagged prompt sections** gave Claude clear boundaries between context, instructions, and constraints. Anthropic's docs confirm this improves parsing for complex prompts.

4. **Fresh session for implementation** (not compacted continuation) eliminated meta-discussion baggage from the research phase.

5. **Compile verification gates** between build steps caught errors early and prevented cascading failures.

6. **Explicit exclusion list** prevented feature creep more effectively than general "keep it simple" instructions.

### What Could Be Improved

1. **Line count estimates were 15% low** (990 estimated vs 1163 actual). The 4-phase activation sequence was larger than anticipated. Future estimates should add 20% buffer for lifecycle code with per-motor loops.

2. **The research prompt could have included a "verify your parameter tables" step** — cross-referencing the Official SDK, K-Scale actuator, and EDULITE_A3 was done but not explicitly mandated.

3. **No unit test prompt was included.** A third agent could have been tasked with writing protocol-layer unit tests (encode/decode round-trips, per-model parameter verification) without hardware.
