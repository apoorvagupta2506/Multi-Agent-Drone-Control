# Multi-Agent Drone Control Simulation — Comprehensive Project Report

> **Author / Organization:** INQUIRELAB (MIT License, © 2025)
> **Repository Root:** `Multi-Agent-Drone-Control/`
> **Language / Platform:** MATLAB

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Tech Stack](#2-tech-stack)
3. [Repository Structure](#3-repository-structure)
4. [Core Concepts & Theory](#4-core-concepts--theory)
5. [Shared Function Modules](#5-shared-function-modules)
6. [Case Study 1 — Basic Flocking with No Obstacle Avoidance (2D)](#6-case-study-1--basic-flocking-with-no-obstacle-avoidance-2d)
7. [Case Study 2 — Multi-Obstacle Avoidance with 8 Agents (2D)](#7-case-study-2--multi-obstacle-avoidance-with-8-agents-2d)
8. [Case Study 3 — Full 3D Swarm with 12 Agents and Spherical Obstacles](#8-case-study-3--full-3d-swarm-with-12-agents-and-spherical-obstacles)
9. [Control Architecture (Unified View)](#9-control-architecture-unified-view)
10. [Simulation Parameters Reference](#10-simulation-parameters-reference)
11. [Outputs and Visualisations](#11-outputs-and-visualisations)
12. [Key Differences Across Case Studies](#12-key-differences-across-case-studies)
13. [How to Run](#13-how-to-run)
14. [License](#14-license)

---

## 1. Project Overview

**Multi-Agent Drone Control** is a MATLAB-based simulation framework that studies the cooperative behaviour of a swarm of Unmanned Aerial Vehicles (UAVs / drones). The project investigates:

- **Flocking / Formation Control** — how multiple drones organise themselves into an optimal spatial formation using Voronoi centroid computation.
- **Formation Reconfiguration** — how the swarm automatically changes its shape when approaching a narrow corridor (like a gap between two buildings).
- **Obstacle Avoidance** — how each drone independently detects and steers around static and moving point obstacles using a potential-field + rotation controller.
- **Inter-Agent Collision Avoidance** — how agents avoid each other in flight.

The project is split into three progressively complex **Case Studies**, each adding more agents, more obstacles, or a higher-dimensional environment.

---

## 2. Tech Stack

| Layer | Technology | Details |
|---|---|---|
| **Programming Language** | MATLAB | All simulation, control logic, and visualisation written in `.m` scripts and functions |
| **Numerical Integration** | Runge-Kutta 4 (RK4) | Implemented manually inside each main script |
| **Spatial Decomposition** | Voronoi Tessellation | MATLAB built-in `voronoi()` for display; custom Lloyd's-algorithm implementation for centroid computation |
| **Coordinate Frames** | Homogeneous Transformation Matrices | Custom `TransMat.m` function; 4x4 SE(3) matrices |
| **Visualisation** | MATLAB `plot3`, `fill3`, `surf`, `voronoi`, `text`, `drawnow` | Real-time animated 3D trajectory video |
| **Data Structures** | MATLAB `cellmat` (cell arrays of matrices) | Used to store per-agent state trajectories |
| **Version Control** | Git | `.git` folder present in the repository root |
| **License** | MIT | Copyright 2025 INQUIRELAB |

> **Note:** No external MATLAB toolboxes are required. The entire project relies only on core MATLAB built-ins.

---

## 3. Repository Structure

```
Multi-Agent-Drone-Control/
|
+-- CASE STUDY 1/             <- Basic flocking, no obstacle avoidance (variable NOA)
|   +-- dynamics_noobs.m      <- Main simulation script (entry point)
|   +-- Centroid_Calculator.m <- Lloyd's algorithm: 2D Voronoi centroid computation
|   +-- Formation_Change.m    <- Lloyd's algorithm: recompute centroids for new formation
|   +-- Dyn.m                 <- Double-integrator drone dynamics function
|   +-- TransMat.m            <- Barrier-frame transformation matrix (I to B)
|   +-- Sat.m                 <- Saturation / clamping function
|   +-- distance.m            <- Euclidean distance helper
|   +-- Coverageplot.m        <- Coverage area visualisation utility
|
+-- CASE STUDY 2/             <- 8 agents, 4 obstacles (3 static + 1 moving), 2D avoidance
|   +-- dynamics_multiobs.m   <- Main simulation script (entry point)
|   +-- Centroid_Calculator.m <- Lloyd's-based centroid computation
|   +-- Formation_Change.m    <- Formation-change logic
|   +-- Dyn.m                 <- Drone dynamics
|   +-- TransMat.m            <- Transformation matrix utility
|   +-- Sat.m                 <- Saturation function
|   +-- distance.m            <- Distance helper
|
+-- CASE STUDY 3/             <- 12 agents, 4 spherical obstacles, full 3D avoidance
|   +-- dynamics_multiobs.m   <- Main simulation script (entry point)
|   +-- Centroid_Calculator.m <- 2D centroid calculator (for barrier projection)
|   +-- Centroid_Calculator_3D.m <- 3D Voronoi centroid calculator (standalone demo)
|   +-- Formation_Change.m    <- Formation-change routine
|   +-- Dyn.m                 <- Drone dynamics
|   +-- TransMat.m            <- Transformation matrix
|   +-- Sat.m                 <- Saturation function
|   +-- distance.m            <- Distance helper
|
+-- .git/                     <- Git version control metadata
+-- LICENSE                   <- MIT License
```

---

## 4. Core Concepts and Theory

### 4.1 Drone Dynamics Model (Double Integrator)

Each drone is modelled as a **3-DOF double integrator**. The state vector for agent `j` is:

```
X_j = [x, y, z, vx, vy, vz]'   (6x1 column vector)
```

The dynamics function (`Dyn.m`) is:

```matlab
function xdot = Dyn(X, U)
    xdot = [X(4:6); U];   % velocity -> position; control input -> acceleration
end
```

Position evolves from velocity; velocity evolves from the control input `U` (acceleration command).

### 4.2 Numerical Integration — RK4

The simulation loop uses a **4th-order Runge-Kutta** (RK4) scheme:

```matlab
f1 = dt * Dyn(X, U);
f2 = dt * Dyn(f1/2 + X, U);
f3 = dt * Dyn(f2/2 + X, U);
f4 = dt * Dyn(f3 + X, U);
X  = X + (f1 + 2*f2 + 2*f3 + f4) / 6;
```

Time step: `dt = 0.1 s`, total simulation time: `tf = 142.5 s`.

### 4.3 Voronoi Tessellation and Centroid Computation (Lloyd's Algorithm)

The target positions for drones are the **Voronoi centroids** of the desired coverage area. `Centroid_Calculator.m` implements a **stochastic Lloyd's algorithm**:

1. **Initialise** generators at current drone positions projected onto a 2D barrier plane.
2. **Sample** random points uniformly from the desired rectangular area (Omega).
3. **Assign** each sample to its nearest generator (builds Voronoi regions Wi).
4. **Update** each generator toward the centroid of its Voronoi region using a weighted average.
5. Repeat for `iteration` iterations.

Update rule:
```
Gens(:,i) = ((alpha1*Ji + beta1)*Gens(:,i) + (alpha2*Ji + beta2)*u) / (Ji + 1)
```
where `u` is the centroid of Wi and `Ji` is a momentum counter.

### 4.4 Coordinate Frame Transformation

`TransMat.m` computes the rigid body transformation matrix from the **Inertial frame (I)** to the **Barrier frame (B)** using Gram-Schmidt orthonormalisation:

```matlab
b1 = (e2 - e1) / norm(e2 - e1);       % first basis vector (along barrier edge)
d  = (el - e1) / norm(el - e1);
h  = d - (b1'*d)*b1;
b2 = h / norm(h);                       % second basis vector (Gram-Schmidt)
b3 = cross(b1, b2);                     % third basis vector (normal to barrier)
```

The 4x4 homogeneous transformation `T_I_to_B` is the inverse of `[b1 b2 b3 e1; 0 0 0 1]`.

### 4.5 Formation / Flocking Controller

Each drone is driven toward its Voronoi centroid target by a **PD formation controller**:

```
Uf = -Kpf * (pos_j - centroid_j) - Kvf * (vel_j - 0)
```

The centroid moves with the flock at constant speed `Vx` in x: `centroid_x = Vx*t + centroid_x0`

### 4.6 Formation Change (Building Corridor)

When the leading drone approaches within 15 m of the building at x=50 m, the y-coordinates of all centroids are compressed to squeeze the swarm through the gap (y=6.5 to y=13.5):

```matlab
if distance_to_building < 15
    CentroidsMat_I(2,j) = CentroidsMat_I(2,j) / 2 + 5;   % compress y
elseif X_mat(1,j) > 100
    CentroidsMat_I(2,j) = CentroidsMat_I0(2,j);            % restore original
end
```

### 4.7 Obstacle Avoidance Controller (Case Studies 2 and 3)

An obstacle is detected if two conditions are simultaneously met:

1. It is within the **detection radius**: `||pos_drone - pos_obs|| < rd + obs_radius`
2. It lies within the **sensor FOV angle** relative to the drone's velocity vector.

Once detected, a **potential field + rotation** control law is applied:

- **Potential gradient (delta_Vp)**: pushes the drone radially away.
- **Rotation gradient (delta_Vr)**: steers the drone tangentially around the obstacle.
- **Combined**: `Uo = -ko1*sum(delta_Vp) - sum(delta_Vr) - ko2*velocity`

The rotation angle `alpha_rotation` scales from 0 to pi/2 as the drone moves from detection zone boundary (rd) toward the safety zone boundary (rs).

**CS2 vs CS3 difference**: CS2 uses 2D (x-y plane) potential field and a 2x2 rotation matrix. CS3 uses fully 3D vectors and a compound 3x3 rotation matrix (Rx * Ry * Rz).

### 4.8 Inter-Agent Collision Avoidance

A reactive repulsion term between every pair of drones:

```
pc = (1 / (||prel|| - rs)^2) * (prel / ||prel||)
uc = -kc1 * pc + kc2 * vrel
Uc = sum of uc over all other agents
```

This keeps drones from getting closer than the safety radius `rs`.

---

## 5. Shared Function Modules

| File | Signature | Purpose |
|---|---|---|
| `Dyn.m` | `xdot = Dyn(X, U)` | Double-integrator dynamics: returns state derivative |
| `TransMat.m` | `Trans = TransMat(e1, e2, el)` | Computes 4x4 inertial-to-barrier frame transformation |
| `Centroid_Calculator.m` | `centroids = Centroid_Calculator(Xmat)` | Stochastic Lloyd's algorithm: finds 2D Voronoi centroids |
| `Formation_Change.m` | `centroids = Formation_Change(Xmat)` | Recomputes centroids for new swarm formation |
| `distance.m` | `dist = distance(xi, xj)` | 3D Euclidean distance utility |
| `Sat.m` | `sat = Sat(Innovation)` | Saturates a diagonal matrix element to [-1, 1] |
| `Centroid_Calculator_3D.m` | standalone demo | Prototype 3D volumetric Voronoi centroid computation |

---

## 6. Case Study 1 — Basic Flocking with No Obstacle Avoidance (2D)

**Entry point:** `CASE STUDY 1/dynamics_noobs.m`

### Goal
Demonstrate basic swarm flocking and **automatic formation change** as drones travel through a corridor between two buildings. No obstacle avoidance logic is active.

### Key Parameters

| Parameter | Value |
|---|---|
| Number of Agents (NOA) | Variable (set globally before running) |
| Simulation Duration | 142.5 s |
| Flock speed | 1 m/s (x-direction) |
| Position gain Kpf | diag([3, 1.5, 3]) |
| Velocity gain Kvf | diag([5, 5, 5]) |
| Building location | x = 50 m |
| Corridor gap | y = 6.5 m to y = 13.5 m (7 m wide) |

### Simulation Flow

```
1. Set parameters
2. Compute Voronoi centroids in Barrier frame (Lloyd's algorithm)
3. Transform centroids to Inertial frame
4. FOR each time step:
   a. Formation Controller (Uf): drive each drone to its centroid
   b. Formation Change: compress y-centroids near building
   c. Collision Avoidance (Uc): inter-agent repulsion
   d. Total control: U = Uf + [Uc; 0]
   e. RK4 integration -> update drone states
   f. Apply building wall hard constraint on y
   g. Update moving obstacle position
   h. Record data
5. Plot Voronoi diagram, animated trajectory, relative distances
```

---

## 7. Case Study 2 — Multi-Obstacle Avoidance with 8 Agents (2D)

**Entry point:** `CASE STUDY 2/dynamics_multiobs.m`

### Goal
Add **full obstacle avoidance** for 3 static + 1 moving cylindrical obstacle, with 8 agents operating at fixed altitude z=5.

### Key Parameters

| Parameter | Value |
|---|---|
| Number of Agents (NOA) | 8 |
| FOV angle | 100 degrees |
| Obstacle detection gains | ko1=5, ko2=1, kox=0.1, koy=0.5, kr=0.5 |
| Collision avoidance gains | kc1=10, kc2=1 |
| Obs.4 start moving | t = 42 s |
| Obs.4 velocity | vx=0.2 m/s, vy=0.05 m/s |

### New Features vs CS1

1. **2D Obstacle Detection** using range AND 2D FOV angle (`atan2`-based)
2. **2D Potential Field + Rotation** obstacle avoidance
3. **Moving Obstacle** (Obs.4) starts at t=42 s
4. **Cylinder Visualisation** using MATLAB `cylinder` + `surf`
5. **Total control** = `U = Uf + [Uo; 0] + [Uc; 0]`

---

## 8. Case Study 3 — Full 3D Swarm with 12 Agents and Spherical Obstacles

**Entry point:** `CASE STUDY 3/dynamics_multiobs.m`

### Goal
Generalise to **full 3D space** with 12 agents and spherical obstacles. FOV detection uses 3D dot product. Avoidance uses compound 3D rotation matrices.

### Key Parameters

| Parameter | Value |
|---|---|
| Number of Agents (NOA) | 12 |
| Centroid samples Snum | 2000 |
| Lloyd iterations | 300 |
| FOV half-angle | 60 degrees (tighter than CS2) |
| Position gain Kpf | diag([3, 3, 3]) |
| Z-axis obstacle gain koz | 0.1 (new axis added) |

### New Features vs CS2

1. **3D FOV detection** using `acos(dot(obs_vec, vel) / (norms))` instead of 2D atan2
2. **3D potential field**: all gradient vectors are 3x1
3. **3D rotation matrix** compound: `Tr = Rx * Ry * Rz`
4. **Spherical obstacle visualisation** via parametric sphere mesh (theta-phi)
5. **3D collision avoidance**: `prel` and `vrel` are full 3-vectors
6. **`Centroid_Calculator_3D.m`**: prototype for 3D volumetric Voronoi coverage

---

## 9. Control Architecture (Unified View)

```
+----------------------------------------------------------------+
|              SIMULATION LOOP (for each time step)              |
|                                                                |
|  +-------------+    +--------------------------------------+   |
|  | Voronoi     |    |          FOR each agent j            |   |
|  | Centroid    |    |                                      |   |
|  | Computation |    |  +----------------------------------+|   |
|  | (Lloyd's)   |--->|  | Formation Controller (PD)        ||   |
|  +-------------+    |  | Uf = -Kpf*(pos-centroid)         ||   |
|                     |  |      -Kvf*(vel-0)                ||   |
|  +-------------+    |  +----------------+-----------------+|   |
|  | Formation   |    |                   |                  |   |
|  | Change      |    |  +----------------v-----------------+|   |
|  | (Building   |--->|  | Obstacle Avoidance (CS2/3 only)  ||   |
|  |  Trigger)   |    |  | Detect (range + FOV)             ||   |
|  +-------------+    |  | Uo = -ko1*dVp - dVr - ko2*vel   ||   |
|                     |  +----------------+-----------------+|   |
|                     |                   |                  |   |
|                     |  +----------------v-----------------+|   |
|                     |  | Inter-Agent Collision Avoidance  ||   |
|                     |  | Uc = sum(-kc1*pc + kc2*vrel)    ||   |
|                     |  +----------------+-----------------+|   |
|                     |                   |                  |   |
|                     |  +----------------v-----------------+|   |
|                     |  | Total Control: U = Uf + Uo + Uc ||   |
|                     |  +----------------+-----------------+|   |
|                     |                   |                  |   |
|                     |  +----------------v-----------------+|   |
|                     |  | RK4 Integration -> X_new         ||   |
|                     |  +----------------------------------+|   |
|                     +--------------------------------------+   |
+----------------------------------------------------------------+
```

---

## 10. Simulation Parameters Reference

| Parameter | Symbol | CS1 | CS2 | CS3 |
|---|---|---|---|---|
| Number of Agents | NOA | variable | 8 | 12 |
| Centroid samples | Snum | 1000 | 1000 | 2000 |
| Lloyd iterations | iteration | 100 | 100 | 300 |
| Flock speed (x) | Vx | 1 m/s | 1 m/s | 1 m/s |
| Detection zone | rd | 2 m | 2 m | 2 m |
| Safety zone | rs | 1 m | 1 m | 1 m |
| Position gain | Kpf | diag(3,1.5,3) | diag(1,1.5,1) | diag(3,3,3) |
| Velocity gain | Kvf | diag(5,5,5) | diag(5,5,5) | diag(5,5,5) |
| Obstacle avoidance | Uo | No | Yes (2D) | Yes (3D) |
| FOV angle | FOV_theta | — | 100 deg | 60 deg |
| Number of obstacles | — | 4 (inactive) | 4 (active) | 4 (active) |
| Obstacle shape | — | — | Cylinders | Spheres |
| Simulation time | tf | 142.5 s | 142.5 s | 142.5 s |
| Time step | dt | 0.1 s | 0.1 s | 0.1 s |

---

## 11. Outputs and Visualisations

Each main script produces the following MATLAB figures:

| Figure | Content |
|---|---|
| Figure 1 (Voronoi) | Initial centroid layout in Barrier frame; shows optimal spatial partitioning |
| Figure 2 (Animation) | Real-time 3D animated trajectory of drone paths, Voronoi overlay, building walls, obstacles |
| Figure 3 (Distances) | Time-series plot of inter-drone distances to verify collision-free operation |

**Visualisation details:**
- Drones: `kx` markers with dotted trail paths (`plot3`)
- Buildings: 2D filled polygons using `fill3` (black)
- Coverage zone: semi-transparent green patch (`fill3`, alpha=0.3)
- CS2 obstacles: vertical cylinders (`surf` on MATLAB `cylinder`)
- CS3 obstacles: spheres (parametric theta-phi meshgrid + `surf`)
- Animation updates every 10 steps or at the first 25 steps
- VideoWriter code is present but commented out for easy video export

---

## 12. Key Differences Across Case Studies

| Feature | CS1 | CS2 | CS3 |
|---|---|---|---|
| Agent count | Variable | 8 | 12 |
| Obstacle avoidance | No | Yes, 2D | Yes, 3D |
| FOV sensor model | No | 2D atan2 | 3D acos/dot |
| Rotation in avoidance | No | 2D (2x2 R) | 3D (3x3 Rx*Ry*Rz) |
| Collision avoidance | 2D | 2D | 3D |
| Obstacle rendering | None | Cylinders | Spheres |
| Centroid samples | 1000 | 1000 | 2000 |
| Lloyd iterations | 100 | 100 | 300 |
| 3D Voronoi prototype | No | No | Yes |

---

## 13. How to Run

### Prerequisites
- MATLAB (any recent version; no special toolboxes required)
- The `cellmat` function must be available (standard MATLAB function)

### Steps

1. Open MATLAB and navigate to the desired Case Study folder.

2. For Case Study 1 only — set the number of agents before running:
   ```matlab
   global NOA
   NOA = 5;   % or any desired number
   ```

3. Run the main script:
   - CS1: run `dynamics_noobs.m`
   - CS2: run `dynamics_multiobs.m` (inside CS2 folder)
   - CS3: run `dynamics_multiobs.m` (inside CS3 folder)

4. MATLAB will compute Voronoi centroids (Lloyd's iterations), then start the animated simulation.

5. To export video, uncomment the `VideoWriter` lines in the animation section.

### Expected Runtime
- CS1: 30 to 60 seconds (depending on NOA)
- CS2: 60 to 90 seconds (8 agents, obstacle avoidance per step)
- CS3: 2 to 4 minutes (12 agents, 3D avoidance, 300 Lloyd iterations)

---

## 14. License

```
MIT License
Copyright (c) 2025 INQUIRELAB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---

*Report generated: September 2026 | Project: Multi-Agent-Drone-Control | Organization: INQUIRELAB*
