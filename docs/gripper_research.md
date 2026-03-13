# Open-Source Robotic Gripper Research

**Date:** 2026-02-27
**Purpose:** Evaluate open-source 1-2 DOF grippers suitable for mounting on a humanoid robot wrist
**Criteria:** Under 500g, electric actuation, public CAD, actively maintained or widely forked

---

## Table of Contents

1. [Summary Comparison Table](#summary-comparison-table)
2. [PincOpen (Pollen Robotics)](#1-pincopen-pollen-robotics)
3. [SSG-48 (Source Robotics)](#2-ssg-48-source-robotics--pcrnjak)
4. [Robo9 Parallel Gripper (for SO-ARM100/101)](#3-robo9-parallel-gripper-for-so-arm100101)
5. [OpenParallelGripper](#4-openparallelgripper)
6. [Actuated UMI Gripper](#5-actuated-umi-gripper)
7. [ALOHA 2 Gripper (Google DeepMind / Stanford)](#6-aloha-2-gripper-google-deepmind--stanford)
8. [Koch v1.1 Gripper](#7-koch-v11-gripper)
9. [SO-ARM100 Built-in Gripper](#8-so-arm100-built-in-gripper)
10. [OpenGrip](#9-opengrip)
11. [MAGPIE Gripper](#10-magpie-gripper)
12. [GELLO Gripper](#11-gello-gripper)
13. [RealMan Two-Finger Gripper](#12-realman-two-finger-gripper)
14. [Compliant Finray-Effect Gripper](#13-compliant-finray-effect-gripper)
15. [Recommendations for Humanoid Wrist Mounting](#recommendations-for-humanoid-wrist-mounting)

---

## Summary Comparison Table

| Gripper | Weight | Stroke | Grip Force | Motor | BOM Cost | GitHub Stars | License | ROS2 |
|---------|--------|--------|------------|-------|----------|-------------|---------|------|
| **PincOpen** | ~150-200g (est) | ~100mm (Reachy ref) | 10-50N (Reachy ref) | Feetech STS3215 | ~EUR 25 | 112 | CC-BY-SA-4.0 | No (LeRobot) |
| **SSG-48** | 400g | 48mm | 5-80N | Spectral Micro BLDC | ~$150-200 (est) | 133 | Apache-2.0 | Yes |
| **Robo9 Parallel** | 250g | 76mm | 150N | Feetech STS3215 | ~$70 | 63 | GPL-3.0 | No |
| **OpenParallelGripper (XL330)** | 270g | 65mm | N/A | Dynamixel XL330-M288 | ~$80-120 | 38 | MIT | No |
| **OpenParallelGripper (SCS)** | 340g | 55mm | N/A | Feetech SCS3045M | ~$60-80 | 38 | MIT | No |
| **Actuated UMI** | ~300g (est) | ~60mm (est) | N/A | Dynamixel XL430-W250 | ~$240-280 | 20 | MIT | No |
| **ALOHA 2** | N/A | ~50mm (est) | 27.9N (follower) | XM430/XC430 | ~$200+ | 2,139 (full project) | MIT | No |
| **Koch v1.1** | ~50g (gripper only) | ~30mm (est) | Low | Dynamixel XL330-M288 | ~$24 (gripper servo) | 3,383 (full project) | MIT | No |
| **SO-ARM100** | ~60g (gripper only) | ~85mm (position units) | Moderate | Feetech STS3215 | ~$10 (gripper only) | 5,560 (full project) | Apache-2.0 | Partial |
| **OpenGrip** | N/A | N/A | N/A | Dynamixel (model TBD) | Low | 77 | None listed | No (ROS compatible) |
| **MAGPIE** | 450g | N/A | 32N (0.08N steps) | Dynamixel AX-12A (x2) | ~$150-200 (est) | 10 | MIT | No (UR5 specific) |
| **GELLO** | ~30g (gripper only) | ~30mm (est) | Low | Dynamixel XL330 | ~$50 (gripper portion) | 262 | MIT | No |
| **RealMan** | ~500g | 65mm | N/A (4kg rated load) | WHJ03 integrated joint | N/A | 17 | OpenAtom HW v1.0 | No |
| **Compliant Finray** | N/A | N/A | N/A | None (finger tips only) | ~$5 (filament) | 5 | BSD-3-Clause | No |

*"est" = estimated from CAD/photos/motor specs where exact values not published*

---

## 1. PincOpen (Pollen Robotics)

**GitHub:** https://github.com/pollen-robotics/PincOpen
**Website:** https://pollen-robotics.github.io/PincOpen/
**Stars:** 112 | **Forks:** 13 | **Last active:** 2026-02-26
**License:** CC-BY-SA-4.0

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | 2-finger parallel jaw |
| Actuation | Feetech STS3215 servo (single) |
| Opening | ~100mm (based on Reachy 2 Pincette reference) |
| Grip force | 10N nominal, 50N peak (Reachy 2 reference) |
| Weight | ~150-200g estimated (much lighter than 400g Reachy due to 3D-printed parts vs machined metal) |
| BOM cost | ~EUR 25 total |
| Material | 3D-printed PLA |
| CAD formats | STL, STEP, Onshape (full parametric model) |

### Key Features
- Derived from Reachy 2's Pincette gripper (EUR 1,700 commercial version) at 1/68th the cost
- Interchangeable fingertips with swappable tip system
- Position-controlled with simulated torque limiting (prevents motor burnout and part damage)
- SO-ARM100 compatible -- designed to mount directly on SO-ARM100/101 arms
- Assembly guide with Jupyter notebooks for motor flashing/validation
- Integrated with LeRobot/pypot ecosystem (Feetech motor support)
- Parallel distal phalanx movement like the production Pincette

### BOM Highlights
- 1x Feetech STS3215 servo (~EUR 12-15)
- 3D-printed structural parts (PLA)
- Standard fasteners and hardware
- Full BOM spreadsheet: https://docs.google.com/spreadsheets/d/1iEKxfsqo3RnKw0QtdLJ2hEtYNDy2LInxrnCFLAhpxHk/

### ROS2 Support
No native ROS2 support. Designed for LeRobot/pypot ecosystem. Would require custom hardware interface.

### Who Uses It
- LeRobot community (HuggingFace)
- SO-ARM100/101 users
- Reachy 2 ecosystem researchers who want a budget alternative

### Assessment for Humanoid Wrist
**Strong candidate.** Very low cost, proven design lineage from commercial product, Feetech STS3215 is the same bus servo protocol used in many low-cost arms. Lightweight. Would need a custom mounting adapter for a humanoid wrist but the Onshape parametric model makes this straightforward.

---

## 2. SSG-48 (Source Robotics / PCrnjak)

**GitHub:** https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper
**Docs:** https://source-robotics.github.io/SSG48-gripper-docs/
**Stars:** 133 | **Forks:** 14 | **Last active:** 2026-02-27
**License:** Apache-2.0

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Adaptive parallel jaw (rack and pinion) |
| Actuation | Spectral Micro BLDC driver (11 pole pair motor) |
| Stroke | 0-48mm |
| Grip force | 5N to 80N (continuously adjustable) |
| Weight | 400g |
| Power supply | 12-24V (operates at 24V) |
| Idle power | 0.5W |
| Communication | CAN bus @ 1 Mbit/s |
| Material | PETG |
| Operating temp | -5 to 65 C |
| CAD formats | STL, STEP (both open source) |

### Key Features
- **Force feedback / force control** -- the headline feature. Current sensing detects contact, adjustable force from 5-80N
- Rack and pinion mechanism: motor is pinion, both jaws ride on racks (synchronized)
- Supports both external and internal grasping modes
- CAN bus interface (same bus protocol as many industrial robots)
- Python API for control
- ROS2 package available
- Dedicated GUI application for configuration and testing
- Interchangeable finger tips (fin-ray soft fingers available as add-on)
- Available pre-assembled or as DIY build

### BOM Highlights
- 1x Spectral Micro BLDC controller (~$83 from Tindie, currently out of stock)
- 1x 11-pole-pair BLDC motor
- 3D-printed PETG parts
- CAN bus termination resistor (120 ohm, factory-included)
- Linear rail/rack hardware
- Total DIY cost estimated ~$150-200

### ROS2 Support
**Yes** -- official ROS2 package available.

### Who Uses It
- Source Robotics PAROL6 robot arm users
- Cobot integrators
- Research labs needing adjustable force grasping
- Hackster.io / Hackaday / Thingiverse / MakerWorld community

### Assessment for Humanoid Wrist
**Strongest overall candidate for our use case.** The CAN bus interface aligns perfectly with our SocketCAN architecture. Force feedback via current sensing is genuinely useful for manipulation tasks. 400g is at our weight limit but acceptable. The 48mm stroke is moderate but sufficient for most manipulation tasks. ROS2 support already exists. Main concern: requires Spectral Micro BLDC controller (currently out of stock, $83 each), and 24V power requirement needs a dedicated rail.

---

## 3. Robo9 Parallel Gripper (for SO-ARM100/101)

**GitHub:** https://github.com/roboninecom/3D-Printed-Parallel-Gripper-for-Robotics-Arms
**Website:** https://robonine.com/building-an-open-source-arm-gripper/
**Stars:** 63 | **Forks:** 14 | **Last active:** 2026-02-23
**License:** GPL-3.0

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Parallel jaw |
| Actuation | Feetech STS3215 (12V, 30 kg-cm torque) |
| Stroke | 76mm |
| Grip force | 150N |
| Speed | 30 mm/s |
| Repeatability | 0.1mm |
| Weight | 250g (PLA, 30% infill) |
| Dimensions | 128 x 109 x 130.5 mm |
| DOF | 1 |
| Assembly time | 30-45 min |
| Print time | 2-4 hours |
| BOM cost | ~$70 |
| CAD formats | STL |

### Key Features
- Designed specifically for SO-ARM100/101 arms
- 12-bit absolute magnetic encoder (from STS3215)
- RS485/TTL communication up to 1 Mbps
- Interchangeable camera mount holders (RealSense D405/D435/D435i/D455, Orbbec Gemini 2, USB cameras)
- Python control software included
- Industrial-grade part numbering system
- Comprehensive assembly guide with images
- Full BOM with vendor links

### ROS2 Support
No native ROS2 package. Python control API provided. Compatible with LeRobot ecosystem.

### Who Uses It
- SO-ARM100/101 builders who want a more capable gripper than the stock design
- LeRobot community

### Assessment for Humanoid Wrist
**Excellent candidate.** Best documented specifications of any gripper in this survey. 250g is very light. 76mm stroke and 150N force are both exceptional for the weight class. The 128x109x130.5mm size is somewhat large for a humanoid wrist, but the design could be scaled down. Uses the same Feetech STS3215 as PincOpen and SO-ARM100. Camera mount system is a bonus for manipulation tasks. GPL-3.0 license may be a concern for some use cases.

---

## 4. OpenParallelGripper

**GitHub:** https://github.com/hygradme/OpenParallelGripper
**Stars:** 38 | **Forks:** 7 | **Last active:** 2025-08-02
**License:** MIT

### Specifications (Two Versions)

| Parameter | SCS3045M Version | XL330 Version |
|-----------|-----------------|---------------|
| Motor | Feetech SCS3045M | Dynamixel XL330-M288-T |
| Stroke | 0-55mm | 0-65mm |
| Weight | ~340g | ~270g |
| Control | Digital input or Modbus-RTU/RS485 | Digital input or Modbus-RTU/RS485 |
| Languages | Python 56.7%, C++ 43.3% | Same |

### Key Features
- Designed for UFACTORY LITE 6 cobot, compatible with xArm series
- Two versions offering servo ecosystem flexibility (Feetech vs Dynamixel)
- XL330 version is "lighter, thinner and cheaper" at cost of reduced rigidity
- Both versions share identical external mounting interface
- Pick-and-place demo videos available

### ROS2 Support
No explicit ROS2 package. Control via digital I/O or Modbus-RTU.

### Who Uses It
- UFACTORY LITE 6 / xArm users
- Industrial cobot integrators

### Assessment for Humanoid Wrist
**Good candidate** especially the XL330 version at 270g. The dual-version approach is interesting -- if we already have Dynamixel infrastructure, the XL330 version integrates cleanly. The 65mm stroke on the XL330 version is solid. MIT license is permissive. Main limitation is that documentation of force capabilities is absent.

---

## 5. Actuated UMI Gripper

**GitHub:** https://github.com/actuated-umi/actuated-umi-gripper
**Website:** https://actuated-umi.github.io/
**Stars:** 20 | **Forks:** 0 | **Last active:** 2025-05-06
**License:** MIT

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Parallel jaw on linear rails |
| Actuation | Dynamixel XL430-W250-T (1.4 N-m stall torque) |
| Rails | MGN7C linear rails, 80mm travel |
| Springs | Constant force coil springs (2x, passive return-to-open) |
| Weight | ~300g estimated |
| Material | 3D-printed PLA (FDM, 0.4mm nozzle) |
| CAD formats | STL (in /3d-printables directory) |
| BOM cost | ~$240-280 (motor ~$80-120, U2D2 ~$60, rails/springs ~$70, hardware ~$30) |

### Key Features
- Actuated version of the original (passive) UMI data collection gripper
- Designed for policy transfer: demonstrations collected with handheld UMI gripper, deployed on actuated version
- Dual linear rail mechanism for smooth, low-friction parallel jaw motion
- Constant force springs for passive compliance and return-to-open behavior
- Belt-driven synchronized finger actuation
- Franka robot integration support
- Custom Dynamixel API (Python, by Tim Schneider)
- Soft TPU finger option for added compliance

### ROS2 Support
No. Custom Python Dynamixel API only. Would require integration work.

### Who Uses It
- UMI (Universal Manipulation Interface) researchers
- Imitation learning / policy transfer labs
- Franka arm users

### Assessment for Humanoid Wrist
**Moderate candidate.** The linear rail design gives very smooth, low-friction motion which is excellent for force-sensitive manipulation and imitation learning. However, the MGN7C rails and constant force springs make it more complex and expensive than simpler designs. The XL430 is heavier (~57g motor alone) than XL330 alternatives. The UMI ecosystem compatibility is only valuable if you plan to use UMI-style data collection. At ~$240-280 it is one of the more expensive options.

---

## 6. ALOHA 2 Gripper (Google DeepMind / Stanford)

**GitHub:** https://github.com/tonyzhaozh/aloha
**Paper:** https://aloha-2.github.io/
**Stars:** 2,139 | **Forks:** 325 | **Last active:** 2024-04-19
**License:** MIT

### Specifications

| Parameter | Leader | Follower |
|-----------|--------|----------|
| Motor | Dynamixel XC430-W150-T | Dynamixel XM430-W350 |
| Grip force | 0.84N opening force | 27.9N closing force |
| Material | 3D-printed PLA | 3D-printed carbon fiber nylon |
| Design | Low-friction rail | Low-friction rail |
| Fingers | Polyurethane grip tape | Polyurethane grip tape, swappable sizes |

### Key Features
- Low-friction rail design reduced operator opening force from 14.68N to 0.84N (leader)
- Follower grippers produce 2x more force than original ALOHA (27.9N vs 12.8N)
- Carbon fiber nylon 3D printing for increased strength/rigidity in follower
- Swappable finger mounts of different sizes
- Full MuJoCo simulation model with system identification
- All hardware designs open-sourced with detailed tutorial
- Massive community adoption (2,139 stars)

### CAD/Hardware
- STL files for 3D printing available in build guide
- Onshape / STEP availability referenced but format details unclear from public docs
- Hardware assembly tutorial via Google Doc

### ROS2 Support
No native ROS2. Uses custom Dynamixel SDK and ACT (Action Chunking with Transformers) framework.

### Who Uses It
- Google DeepMind researchers
- Stanford robotics labs
- Bimanual manipulation researchers worldwide
- ALOHA / Mobile ALOHA / ALOHA Unleashed users

### Assessment for Humanoid Wrist
**Moderate candidate.** The low-friction rail design is excellent, and the community is enormous. However, ALOHA is designed as a complete bimanual system, not a standalone gripper. Extracting just the gripper design requires significant effort. The XM430-W350 follower motor is relatively heavy (~82g motor alone) and expensive (~$250). The 27.9N grip force is moderate. Best suited if you want to eventually integrate ALOHA-style teleoperation/imitation learning.

---

## 7. Koch v1.1 Gripper

**GitHub:** https://github.com/AlexanderKoch-Koch/low_cost_robot
**Stars:** 3,383 | **Forks:** 299 | **Last active:** 2024-09-21
**License:** MIT

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Simple parallel jaw |
| Actuation | Dynamixel XL330-M288-T (0.52 N-m stall torque at 5V) |
| Motor weight | 18g per motor |
| Stroke | ~30mm estimated (small gripper) |
| Grip force | Low (limited by XL330 torque) |
| Material | 3D-printed PLA |
| CAD formats | STL (in hardware/follower/stl/) |
| BOM cost | ~$24 per motor (gripper uses 1 XL330) |

### Key Features
- Minimalist design -- part of the famous "low cost robot" arm project
- Leader arm uses trigger handle instead of gripper for ergonomic teleoperation
- Follower gripper is the actual parallel jaw for manipulation
- "Only the moving part of the gripper needs supports" during 3D printing
- XL330 motors are extremely lightweight (18g each)
- 5V operation (can run from USB power)
- Massive community (3,383 stars, one of the most popular robot arm projects ever)

### ROS2 Support
No. Pure Python control via Dynamixel SDK.

### Who Uses It
- The original "low cost robot arm" community
- LeRobot / Hugging Face ecosystem
- Robotis official partner (sells as a kit)

### Assessment for Humanoid Wrist
**Limited.** The Koch gripper is designed to be the simplest possible end-effector for a tabletop arm. Small stroke, low force, minimal features. However, the XL330-M288 motor itself is an excellent choice for lightweight applications (18g, 5V, 0.52 N-m). The gripper design could serve as a starting point for a custom lightweight gripper.

---

## 8. SO-ARM100 Built-in Gripper

**GitHub:** https://github.com/TheRobotStudio/SO-ARM100
**Stars:** 5,560 | **Forks:** 483 | **Last active:** 2026-02-26
**License:** Apache-2.0

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Parallel jaw (rigid) or compliant (TPU) |
| Actuation | Feetech STS3215 (12V: 30 kg-cm, 7.4V: 16.5 kg-cm) |
| Position range | 0.0 (closed) to 0.085 (fully open) |
| Material | PLA (rigid) or TPU 95A (compliant) |
| Weight | ~60g (gripper only, estimated) |
| CAD formats | STL |

### Key Features
- **Most popular open-source robot arm project** (5,560 stars)
- Two gripper variants: rigid PLA and compliant TPU
- Compliant gripper uses hollowed-out cavities with internal rib reinforcement
- Drop-in swap between rigid and compliant versions (identical mounting interface)
- Multiple community gripper variants (symmetrical, precise, parallel with camera holder)
- 12-bit absolute magnetic encoder for position feedback
- RS485/TTL UART communication
- Ecosystem of compatible grippers via LeRobotDepot

### Community Gripper Variants
1. **Standard rigid gripper** -- PLA, hard contact surfaces
2. **Compliant gripper** -- TPU 95A, passive compliance
3. **Precise gripper** -- community variant for finer manipulation
4. **Symmetrical gripper** -- community variant
5. **PincOpen** -- Pollen Robotics' drop-in upgrade
6. **Robo9 Parallel Gripper** -- high-force aftermarket gripper

### ROS2 Support
Partial -- the SO-ARM100 has some ROS2 integration via community packages (max_effort parameter available in ROS2 action interface for gripper force control).

### Who Uses It
- By far the largest community of any open-source robot arm
- LeRobot / Hugging Face primary hardware platform
- Educational institutions worldwide

### Assessment for Humanoid Wrist
**The gripper itself is too simple** for humanoid use (tiny jaws, low force, no feedback). However, the STS3215 motor and the mounting/control patterns are well-proven. The SO-ARM100 ecosystem is where PincOpen and Robo9 grippers live, so their compatibility is a plus. The real value is the massive community and parts availability.

---

## 9. OpenGrip

**GitHub:** https://github.com/clayhaight01/OpenGrip
**Stars:** 77 | **Forks:** 4 | **Last active:** 2024-03-02
**License:** Not specified

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Parallel jaw with constant force springs |
| Actuation | Dynamixel servo (specific model not documented) |
| Custom parts | 5 total |
| Weight | Not published |
| Stroke | Not published |
| Grip force | "Comparable to grippers costing up to 50x more" |
| CAD formats | Available in /CAD files directory |

### Key Features
- Extreme simplicity: only 5 custom parts
- No belts, linkages, or gears
- Constant force springs for collision protection (passive compliance)
- Adjustable compliance via different winch materials (nylon, rubber, etc.)
- Dynamixel servo for ROS-compatible control
- Assembly instructions via Google Slides
- BOM via Google Sheets

### ROS2 Support
Compatible with ROS ecosystem via Dynamixel servo. No dedicated ROS2 package.

### Who Uses It
- Robotics researchers wanting a dead-simple gripper
- ROS users with Dynamixel infrastructure

### Assessment for Humanoid Wrist
**Interesting design philosophy** but under-documented. The constant force spring approach for collision protection is clever and relevant for humanoid manipulation. The simplicity (5 parts, no gears) is appealing. However, no published specs for weight, stroke, or force makes it hard to evaluate. Project appears dormant since March 2024. No license specified.

---

## 10. MAGPIE Gripper

**GitHub:** https://github.com/correlllab/MAGPIE
**Paper:** https://arxiv.org/pdf/2402.06018
**Stars:** 10 | **Forks:** 3 | **Last active:** 2025-12-16
**License:** MIT

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Dual-motor 4-bar linkage |
| Actuation | 2x Dynamixel AX-12A |
| Grip force | Up to 32N, controllable in 0.08N steps |
| Weight | 450g |
| DOF | 2 (independent finger control) |
| Vision | Intel RealSense D405 in-palm camera |
| Material | 3D-printed |
| BOM cost | ~$150-200 estimated |

### Key Features
- **Force control with 0.08N resolution** -- by far the finest force control of any gripper in this survey
- **In-palm 3D perception** with Intel RealSense D405 camera
- Dual motor independent finger operation (can apply asymmetric forces)
- 4-bar linkage mechanism (more compact than full parallel jaw)
- Complete autonomous manipulation pipeline: object detection, segmentation, point cloud processing, force-controlled manipulation, symbolic re-planning
- Designed for UR5 cobot integration
- Built with COTS components (AX-12A servos are cheap and widely available, ~$12-15 each)

### ROS2 Support
No. Custom Python stack designed for UR5. However, the software architecture is modular.

### Who Uses It
- Correll Lab, University of Colorado Boulder
- Autonomous manipulation researchers

### Assessment for Humanoid Wrist
**Interesting but heavy.** At 450g it's at our weight limit. The 4-bar linkage is more compact than parallel jaw alternatives. The 0.08N force resolution is exceptional and useful for delicate manipulation. The AX-12A servos are old and slow but extremely cheap. The in-palm camera is a unique feature. The dual-motor independent finger control enables object centering and active exploration. However, AX-12A servos are outdated (10-bit position, TTL only) and the 32N max force is moderate. The full software stack (perception + planning) adds value beyond the hardware.

---

## 11. GELLO Gripper

**GitHub:** https://github.com/wuphilipp/gello_mechanical
**Website:** https://wuphilipp.github.io/gello_site/
**Stars:** 262 | **Forks:** 32 | **Last active:** 2025-11-08
**License:** MIT

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Simple trigger/jaw for teleoperation |
| Actuation | Dynamixel XL330-M288-T or XL330-M077-T |
| Motor weight | 18g |
| Motor dimensions | 20 x 34 x 26 mm |
| Motor torque | 0.52 N-m (M288) at 5V |
| CAD formats | OpenSCAD (primary), STL |
| BOM cost | ~$300 total system (under $50 for gripper portion) |

### Key Features
- GELLO is primarily a teleoperation framework, not a standalone gripper
- The gripper is a shared design across all robot variants (Franka, UR5, xArm)
- Kinematically equivalent to target robot arm
- Extremely lightweight due to XL330 motors
- Full open-source mechanical and software
- Now sold as official Robotis bundles (GELLO Franka Bundle, GELLO UR5 Bundle, GELLO xArm Bundle)
- Assembly takes ~30 minutes

### ROS2 Support
No native ROS2. Custom Python SDK.

### Who Uses It
- Teleoperation researchers
- Franka / UR5 / xArm users
- Robotis commercial customers

### Assessment for Humanoid Wrist
**Not directly applicable** -- GELLO grippers are teleop leader devices, not follower grippers for manipulation. However, the gripper mechanism design and XL330 motor integration are clean references for building a lightweight gripper. The OpenSCAD parametric design is unusual but enables programmatic customization.

---

## 12. RealMan Two-Finger Gripper

**GitHub:** https://github.com/RealManRobot/Two-Finger-Gripper
**Stars:** 17 | **Forks:** 5 | **Last active:** 2025-01-16
**License:** OpenAtom Open Hardware License v1.0

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Parallel jaw |
| Actuation | WHJ03 integrated joint |
| Opening | 65mm |
| Open/close time | 0.4s |
| Weight | ~500g |
| Rated load | 4kg |
| Max load | 5kg |
| Communication | RS485 (Modbus RTU) |
| Power | 24V DC |
| Connector | 6-pin aviation plug + 6-pin PH2.0 terminal |

### CAD / Hardware
- 3D CAD models (RMG24 format)
- Schematic diagrams (Altium Designer format)
- PCB layouts with Gerber files
- Component placement diagrams
- BOM documentation
- Firmware source code (C/C++: gripper_app, gripper_boot, gripper_tool)
- Windows host software (standalone executable, no installation needed)

### Key Features
- Complete open hardware design including PCB/electronics (rare among open-source grippers)
- WHJ03 integrated joint with custom control board
- Torque/speed control, position management, firmware OTA updates
- Three C source code modules provided
- Host computer GUI for parameter configuration
- High load capacity for its size (4-5kg)

### ROS2 Support
No explicit ROS2 support. RS485 Modbus RTU interface would need a driver.

### Who Uses It
- RealMan robot arm users
- Industrial integrators in China

### Assessment for Humanoid Wrist
**Mixed.** At 500g it's right at our weight limit. The 65mm stroke and 4kg load capacity are impressive. The complete open hardware (including PCB, firmware, host software) is the most thorough of any project in this survey. The WHJ03 integrated joint is a self-contained actuator module which simplifies integration. However, the 24V power requirement and RS485 Modbus protocol add complexity. The OpenAtom license is uncommon and needs legal review. Documentation primarily in Chinese.

---

## 13. Compliant Finray-Effect Gripper

**GitHub:** https://github.com/richardhartisch/compliantfinray
**Stars:** 5 | **Forks:** 0 | **Last active:** 2025-09-10
**License:** BSD-3-Clause

### Specifications

| Parameter | Value |
|-----------|-------|
| Type | Compliant fin-ray fingers (not a complete gripper) |
| Actuation | None (finger tips designed for Weiss IEG 76-030 gripper module) |
| Material | 3D-printed (varying infill density: 10-30%, direction: 0-40 degrees) |
| Slicer | Ultimaker Cura with specific settings |
| CAD formats | IPT, STP, STL |
| Print settings | Wall line count: 1, infill: lines, no top/bottom layers |

### Key Features
- Designed for high-speed robotic assembly of electrical components
- Parametric compliance through infill direction and density
- FEA (Finite Element Analysis) models included
- Cura "support blocker" technique for multi-parameter printing within single part
- Research-quality design with rigorous characterization

### Assessment for Humanoid Wrist
**Not a standalone gripper** -- these are compliant finger tips for an existing industrial gripper (Weiss IEG 76-030). However, the fin-ray finger designs and FEA models could be adapted as soft finger pads for any of the other grippers in this survey. The parametric infill approach to tuning compliance is a clever technique worth borrowing. BSD-3 license is very permissive.

Additional fin-ray resources:
- SSG-48 has optional fin-ray finger add-ons
- Thingiverse has several standalone fin-ray gripper designs (e.g., thing:4894257)
- Source Robotics blog has a good overview: https://source-robotics.com/blogs/blog/soft-robotic-grippers-fin-ray-effect

---

## Recommendations for Humanoid Wrist Mounting

### Tier 1: Best Candidates

**1. SSG-48 (Source Robotics)** -- Best overall
- Pro: Force feedback, CAN bus (matches our SocketCAN architecture), ROS2 support, 80N force, well-documented
- Con: 400g weight, 24V power, Spectral Micro BLDC controller availability, 48mm stroke is moderate
- Effort: Medium (CAN driver exists, ROS2 package exists, needs wrist adapter)

**2. Robo9 Parallel Gripper** -- Best value for specs
- Pro: 250g, 76mm stroke, 150N force, $70 BOM, excellent documentation, camera mount system
- Con: GPL-3.0 license, Feetech STS3215 requires TTL/RS485 bus (not CAN), 128x109x130mm is large
- Effort: Medium (needs Feetech driver, wrist adapter, and possibly size reduction)

**3. PincOpen (Pollen Robotics)** -- Best budget option
- Pro: EUR 25, proven Reachy 2 lineage, Onshape parametric model, SO-ARM100 ecosystem
- Con: Specs not fully documented, Feetech STS3215 (not CAN), no ROS2
- Effort: Low-Medium (simple design, needs Feetech driver and wrist adapter)

### Tier 2: Worth Considering

**4. OpenParallelGripper (XL330 version)** -- If we want Dynamixel ecosystem
- 270g, 65mm stroke, MIT license, proven on UFACTORY cobots
- Needs force characterization and ROS2 integration

**5. ALOHA 2 Gripper** -- If we want imitation learning compatibility
- Huge community, proven design, carbon fiber nylon follower
- Complex extraction from full system, expensive motors, moderate force

### Tier 3: Niche Use Cases

**6. Actuated UMI Gripper** -- For UMI/imitation learning pipeline
**7. MAGPIE** -- For precision force control + vision-in-hand
**8. RealMan Two-Finger** -- For maximum payload capacity (4-5kg)

### Key Integration Considerations for SteveROS

1. **Communication bus**: Our robot uses SocketCAN (can0). The SSG-48 with CAN bus is the only gripper that natively matches. All Feetech (STS3215) and Dynamixel grippers use TTL/RS485 serial, requiring either a separate serial interface or a CAN-to-serial bridge.

2. **Power**: Our motors run on 24V through Robstride RS-series drivers. The SSG-48 also runs on 24V. Feetech STS3215 needs 7.4-12V. Dynamixel XL330 needs 5V, XL430/XM430 need 12V.

3. **Weight budget**: A humanoid wrist typically has very limited payload. Under 300g for the gripper is strongly preferred. This favors PincOpen, Robo9, and OpenParallelGripper (XL330).

4. **ros2_control integration**: The SSG-48 has ROS2 support. Everything else would need a custom hardware interface plugin (similar to what we already wrote for steveros_hardware).

5. **Force sensing**: Only SSG-48 and MAGPIE have real force feedback. Others rely on motor current limiting or compliance. For autonomous manipulation, force feedback is highly valuable.

---

## Source Links

- [PincOpen GitHub](https://github.com/pollen-robotics/PincOpen)
- [PincOpen Website](https://pollen-robotics.github.io/PincOpen/)
- [SSG-48 GitHub](https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper)
- [SSG-48 Docs](https://source-robotics.github.io/SSG48-gripper-docs/)
- [SSG-48 on Hackaday](https://hackaday.io/project/202979-ssg-48-adaptive-electric-gripper)
- [Spectral Micro BLDC on Tindie](https://www.tindie.com/products/sourcerobotics/spectral-micro-bldc-controller/)
- [Robo9 Parallel Gripper GitHub](https://github.com/roboninecom/3D-Printed-Parallel-Gripper-for-Robotics-Arms)
- [Robo9 Blog Post](https://robonine.com/building-an-open-source-arm-gripper/)
- [OpenParallelGripper GitHub](https://github.com/hygradme/OpenParallelGripper)
- [Actuated UMI Gripper GitHub](https://github.com/actuated-umi/actuated-umi-gripper)
- [Actuated UMI Website](https://actuated-umi.github.io/)
- [ALOHA GitHub](https://github.com/tonyzhaozh/aloha)
- [ALOHA 2 Paper](https://aloha-2.github.io/assets/aloha2.pdf)
- [ALOHA 2 Website](https://aloha-2.github.io/)
- [Koch v1.1 GitHub](https://github.com/AlexanderKoch-Koch/low_cost_robot)
- [Koch v1.1 HuggingFace Docs](https://huggingface.co/docs/lerobot/en/koch)
- [SO-ARM100 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- [SO-ARM100 Grippers DeepWiki](https://deepwiki.com/TheRobotStudio/SO-ARM100/2.4.3-grippers-and-end-effectors)
- [LeRobotDepot GitHub](https://github.com/maximilienroberti/lerobotdepot)
- [OpenGrip GitHub](https://github.com/clayhaight01/OpenGrip)
- [MAGPIE GitHub](https://github.com/correlllab/MAGPIE)
- [MAGPIE Paper](https://arxiv.org/pdf/2402.06018)
- [GELLO Mechanical GitHub](https://github.com/wuphilipp/gello_mechanical)
- [GELLO Website](https://wuphilipp.github.io/gello_site/)
- [RealMan Two-Finger Gripper GitHub](https://github.com/RealManRobot/Two-Finger-Gripper)
- [Compliant Finray GitHub](https://github.com/richardhartisch/compliantfinray)
- [Reachy 2 Gripper Specs](https://docs.pollen-robotics.com/hardware-guide/specifications/grippers/)
- [Source Robotics Fin-Ray Blog](https://source-robotics.com/blogs/blog/soft-robotic-grippers-fin-ray-effect)
- [Feetech STS3215 on Alibaba](https://www.alibaba.com/product-detail/Feetech-STS3215-SO-ARM100-Servo-12V_1601292634404.html)
- [Dynamixel XL330-M288 Specs](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)
- [Dynamixel XL430-W250 Specs](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
