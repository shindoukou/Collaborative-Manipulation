# Collaborative Manipulation with Dual Arms in Narrow Spaces

<p align="center"><strong>A visual project page for safe, coordinated dual-arm grasping and transport in constrained workspaces.</strong></p>

<p align="center">
  <img src="assets/paper/systemV7.png" alt="Overview of the proposed dual-arm manipulation framework" width="100%">
</p>

## Code snapshot

The initial CM workspace snapshot is in [`CM_ws/`](CM_ws/). See the [workspace guide](CM_ws/README_CN.md) and [snapshot scope and validation notes](CM_ws/SNAPSHOT_20260920.md) before running it.

## Overview

This work presents an integrated framework for dual-arm manipulation in narrow spaces. It combines rapid constrained trajectory generation, distributed nonlinear model predictive control (DNMPC), and bilateral position-force control with adaptive impedance. Simulation and real-world experiments cover grasping, transporting, placing, and pouring with rigid, soft, irregular, and deformable targets.

本项目面向狭窄空间中的双臂协同操作，重点展示轨迹生成、分布式非线性模型预测控制和双边位置-力控制。仓库主页仅保留论文的核心概述，但完整展示论文中实际使用的全部图片。

### Main contributions

- A rapid trajectory-generation method that jointly satisfies collision-avoidance, kinematic, and task constraints while maintaining spatio-temporal consistency.
- A distributed NMPC controller that coordinates both arms through predicted-state exchange.
- A bilateral position-force controller with adaptive impedance for explicit internal-force regulation across targets with different stiffness.

**Keywords:** dual-arm system, trajectory generation, distributed nonlinear model predictive control, position-force control.

## Manipulation concept and kinematics

### Grasp and transport phases
<p align="center">
  <img src="assets/paper/Grasp3.png" alt="Manipulation phase 1" width="49%">
  <img src="assets/paper/Trasnport3.png" alt="Manipulation phase 2" width="49%">
</p>

### Coordinate system

<p align="center">
  <img src="assets/paper/coordinate-system.png" alt="Dual-arm coordinate system" width="72%">
</p>

## Algorithm design

### Trajectory generation

<p align="center">
  <img src="assets/paper/algo1_9.png" alt="Trajectory generation algorithm" width="92%">
</p>

#### Constraints in grasping and transport

<p align="center">
  <img src="assets/paper/constraint_grasp6.png" alt="Constraint model 1" width="49%">
  <img src="assets/paper/constraint_transport6.png" alt="Constraint model 2" width="49%">
</p>

### Distributed nonlinear model predictive control

<p align="center">
  <img src="assets/paper/Visicontrolblock6.png" alt="Distributed NMPC control architecture" width="92%">
</p>

### Bilateral position-force control

<p align="center">
  <img src="assets/paper/force7.png" alt="Force-control model 1" width="49%">
  <img src="assets/paper/forcereview_v2.png" alt="Force-control model 2" width="49%">
</p>

#### Adaptive impedance and desired-force design

<p align="center">
  <img src="assets/paper/impedance-adaptation.jpg" alt="Adaptive force design 1" width="32%">
  <img src="assets/paper/Visio-Desired_Force_why_v2_a.png" alt="Adaptive force design 2" width="32%">
  <img src="assets/paper/Visio-Desired_Force_why_v3_b.png" alt="Adaptive force design 3" width="32%">
</p>

## Simulation

### Ball

<p align="center">
  <img src="assets/paper/simulation/ball1.jpg" alt="Ball simulation frame 1" width="15.7%">
  <img src="assets/paper/simulation/ball2.jpg" alt="Ball simulation frame 2" width="15.7%">
  <img src="assets/paper/simulation/ball3.jpg" alt="Ball simulation frame 3" width="15.7%">
  <img src="assets/paper/simulation/ball4.jpg" alt="Ball simulation frame 4" width="15.7%">
  <img src="assets/paper/simulation/ball5.jpg" alt="Ball simulation frame 5" width="15.7%">
  <img src="assets/paper/simulation/ball6.jpg" alt="Ball simulation frame 6" width="15.7%">
</p>

### Box

<p align="center">
  <img src="assets/paper/simulation/box1.jpg" alt="Box simulation frame 1" width="15.7%">
  <img src="assets/paper/simulation/box2.jpg" alt="Box simulation frame 2" width="15.7%">
  <img src="assets/paper/simulation/box3.jpg" alt="Box simulation frame 3" width="15.7%">
  <img src="assets/paper/simulation/box4.jpg" alt="Box simulation frame 4" width="15.7%">
  <img src="assets/paper/simulation/box5.jpg" alt="Box simulation frame 5" width="15.7%">
  <img src="assets/paper/simulation/box6.jpg" alt="Box simulation frame 6" width="15.7%">
</p>

## Experimental platform and procedure

<p align="center">
  <img src="assets/paper/real3.png" alt="Real-world experimental platform" width="92%">
</p>

### Four-stage manipulation procedure

<p align="center">
  <img src="assets/paper/Stage1.png" alt="Manipulation stage 1" width="24%">
  <img src="assets/paper/Stage2.png" alt="Manipulation stage 2" width="24%">
  <img src="assets/paper/Stage3.png" alt="Manipulation stage 3" width="24%">
  <img src="assets/paper/Stage4.png" alt="Manipulation stage 4" width="24%">
</p>

## Real-world manipulation sequences

### Ball

<p align="center">
  <img src="assets/paper/FRpic/ball/1.jpg" alt="Ball experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/ball/2.jpg" alt="Ball experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/ball/3.jpg" alt="Ball experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/ball/4.jpg" alt="Ball experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/ball/5.jpg" alt="Ball experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/ball/6.jpg" alt="Ball experiment frame 6" width="15.7%">
</p>

### Box

<p align="center">
  <img src="assets/paper/FRpic/box/1.jpg" alt="Box experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/box/2.jpg" alt="Box experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/box/3.jpg" alt="Box experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/box/4.jpg" alt="Box experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/box/5.jpg" alt="Box experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/box/6.jpg" alt="Box experiment frame 6" width="15.7%">
</p>

### Basketball

<p align="center">
  <img src="assets/paper/FRpic/basketball/1.jpg" alt="Basketball experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/basketball/2.jpg" alt="Basketball experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/basketball/3.jpg" alt="Basketball experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/basketball/4.jpg" alt="Basketball experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/basketball/5.jpg" alt="Basketball experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/basketball/6.jpg" alt="Basketball experiment frame 6" width="15.7%">
</p>

### Foam frame

<p align="center">
  <img src="assets/paper/FRpic/softbox/1.jpg" alt="Foam-frame experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/softbox/2.jpg" alt="Foam-frame experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/softbox/3.jpg" alt="Foam-frame experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/softbox/4.jpg" alt="Foam-frame experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/softbox/5.jpg" alt="Foam-frame experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/softbox/6.jpg" alt="Foam-frame experiment frame 6" width="15.7%">
</p>

### Plush toy

<p align="center">
  <img src="assets/paper/FRpic/mango/1.jpg" alt="Plush-toy experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/mango/2.jpg" alt="Plush-toy experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/mango/3.jpg" alt="Plush-toy experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/mango/4.jpg" alt="Plush-toy experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/mango/5.jpg" alt="Plush-toy experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/mango/6.jpg" alt="Plush-toy experiment frame 6" width="15.7%">
</p>

### L-shaped wire mesh

<p align="center">
  <img src="assets/paper/FRpic/L/1.jpg" alt="L-shaped wire-mesh experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/L/2.jpg" alt="L-shaped wire-mesh experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/L/3.jpg" alt="L-shaped wire-mesh experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/L/4.jpg" alt="L-shaped wire-mesh experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/L/5.jpg" alt="L-shaped wire-mesh experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/L/6.jpg" alt="L-shaped wire-mesh experiment frame 6" width="15.7%">
</p>

### Clothes-filled bag

<p align="center">
  <img src="assets/paper/FRpic/cloth/1.jpg" alt="Clothes-filled-bag experiment frame 1" width="15.7%">
  <img src="assets/paper/FRpic/cloth/2.jpg" alt="Clothes-filled-bag experiment frame 2" width="15.7%">
  <img src="assets/paper/FRpic/cloth/3.jpg" alt="Clothes-filled-bag experiment frame 3" width="15.7%">
  <img src="assets/paper/FRpic/cloth/4.jpg" alt="Clothes-filled-bag experiment frame 4" width="15.7%">
  <img src="assets/paper/FRpic/cloth/5.jpg" alt="Clothes-filled-bag experiment frame 5" width="15.7%">
  <img src="assets/paper/FRpic/cloth/6.jpg" alt="Clothes-filled-bag experiment frame 6" width="15.7%">
</p>

## Quantitative results

### Ball: distance, force, constraints, and position

<p align="center">
  <img src="assets/paper/EXP08/ball/distance_during_grasping.png" alt="Ball distance result 1" width="49%">
  <img src="assets/paper/EXP08/ball/distance_during_transporting.png" alt="Ball distance result 2" width="49%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/ball/d_f_legend.png" alt="Legend for distance and force plots" width="72%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/ball/force_during_grasping.png" alt="Ball force result 1" width="49%">
  <img src="assets/paper/EXP08/ball/force_during_transporting.png" alt="Ball force result 2" width="49%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/ball/4bc.png" alt="Ball constraint result 1" width="24%">
  <img src="assets/paper/EXP08/ball/4de.png" alt="Ball constraint result 2" width="24%">
  <img src="assets/paper/EXP08/ball/5bc.png" alt="Ball constraint result 3" width="24%">
  <img src="assets/paper/EXP08/ball/5de.png" alt="Ball constraint result 4" width="24%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/ball/position_legend.png" alt="Legend for ball position plots" width="72%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/ball/position_during_grasping.png" alt="Ball position result 1" width="49%">
  <img src="assets/paper/EXP08/ball/position_during_transporting.png" alt="Ball position result 2" width="49%">
</p>

### Box: distance, force, constraints, and position

<p align="center">
  <img src="assets/paper/EXP08/box/distance_during_grasping.png" alt="Box distance result 1" width="49%">
  <img src="assets/paper/EXP08/box/distance_during_transporting.png" alt="Box distance result 2" width="49%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/box/force_during_grasping.png" alt="Box force result 1" width="49%">
  <img src="assets/paper/EXP08/box/force_during_transporting.png" alt="Box force result 2" width="49%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/box/4bc.png" alt="Box constraint result 1" width="49%">
  <img src="assets/paper/EXP08/box/5bc.png" alt="Box constraint result 2" width="49%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/box/position_legend.png" alt="Legend for box position plots" width="72%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/box/position_during_grasping.png" alt="Box position result 1" width="49%">
  <img src="assets/paper/EXP08/box/position_during_transporting.png" alt="Box position result 2" width="49%">
</p>

### Internal-force results for additional targets

<p align="center">
  <img src="assets/paper/EXP08/basketball/force_during_transporting.png" alt="Additional-target force result 1" width="19.2%">
  <img src="assets/paper/EXP08/softbox/force_during_transporting.png" alt="Additional-target force result 2" width="19.2%">
  <img src="assets/paper/EXP08/mango/force_during_transporting.png" alt="Additional-target force result 3" width="19.2%">
  <img src="assets/paper/EXP08/L/force_during_transporting.png" alt="Additional-target force result 4" width="19.2%">
  <img src="assets/paper/EXP08/cloth/force_during_transporting.png" alt="Additional-target force result 5" width="19.2%">
</p>

<p align="center">
  <img src="assets/paper/EXP08/cloth/d_f_legend2.png" alt="Legend for additional-target force plots" width="72%">
</p>

## Fault-tolerance demonstration

### Left-arm interruption

<p align="center">
  <img src="assets/paper/FRpic/fault/left/1.jpg" alt="Left-arm interruption frame 1" width="32%">
  <img src="assets/paper/FRpic/fault/left/2.jpg" alt="Left-arm interruption frame 2" width="32%">
  <img src="assets/paper/FRpic/fault/left/3.jpg" alt="Left-arm interruption frame 3" width="32%">
</p>

### Right-arm interruption

<p align="center">
  <img src="assets/paper/FRpic/fault/right/1.jpg" alt="Right-arm interruption frame 1" width="32%">
  <img src="assets/paper/FRpic/fault/right/2.jpg" alt="Right-arm interruption frame 2" width="32%">
  <img src="assets/paper/FRpic/fault/right/3.jpg" alt="Right-arm interruption frame 3" width="32%">
</p>

### Failure cases

<p align="center">
  <img src="assets/paper/FRpic/fault/leftfail/1.jpg" alt="Fault failure case 1" width="49%">
  <img src="assets/paper/FRpic/fault/rightfail/1.jpg" alt="Fault failure case 2" width="49%">
</p>

### Force response

<p align="center">
  <img src="assets/paper/EXP08/fault/d_f_legend.png" alt="Fault-tolerance force result 1" width="49%">
  <img src="assets/paper/EXP08/fault/force_during_transporting.png" alt="Fault-tolerance force result 2" width="49%">
</p>

## Additional tasks

### Placing a ball

<p align="center">
  <img src="assets/paper/FRpic/placeball/1.jpg" alt="Ball-placing frame 1" width="15.7%">
  <img src="assets/paper/FRpic/placeball/2.jpg" alt="Ball-placing frame 2" width="15.7%">
  <img src="assets/paper/FRpic/placeball/3.jpg" alt="Ball-placing frame 3" width="15.7%">
  <img src="assets/paper/FRpic/placeball/4.jpg" alt="Ball-placing frame 4" width="15.7%">
  <img src="assets/paper/FRpic/placeball/5.jpg" alt="Ball-placing frame 5" width="15.7%">
  <img src="assets/paper/FRpic/placeball/6.jpg" alt="Ball-placing frame 6" width="15.7%">
</p>

### Pouring with a box

<p align="center">
  <img src="assets/paper/FRpic/pourbox/1.jpg" alt="Box-pouring frame 1" width="15.7%">
  <img src="assets/paper/FRpic/pourbox/2.jpg" alt="Box-pouring frame 2" width="15.7%">
  <img src="assets/paper/FRpic/pourbox/3.jpg" alt="Box-pouring frame 3" width="15.7%">
  <img src="assets/paper/FRpic/pourbox/4.jpg" alt="Box-pouring frame 4" width="15.7%">
  <img src="assets/paper/FRpic/pourbox/5.jpg" alt="Box-pouring frame 5" width="15.7%">
  <img src="assets/paper/FRpic/pourbox/6.jpg" alt="Box-pouring frame 6" width="15.7%">
</p>

### Task definitions

<p align="center">
  <img src="assets/paper/placeball_v2.png" alt="Additional task definition 1" width="49%">
  <img src="assets/paper/pourbox_v1.png" alt="Additional task definition 2" width="49%">
</p>

### Force results

<p align="center">
  <img src="assets/paper/EXP08/placeball/force_during_placing.png" alt="Additional task force result 1" width="49%">
  <img src="assets/paper/EXP08/pourbox/force_during_pouring.png" alt="Additional task force result 2" width="49%">
</p>

## Image inventory

This page displays all **125 unique image files** actively referenced by the manuscript source. The source-to-gallery mapping is recorded in [`assets/paper/manifest.json`](assets/paper/manifest.json).

---

This repository is a concise visual companion to the manuscript; it intentionally does not reproduce the complete paper text.
