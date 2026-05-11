# Two-Tank Water System Simulation

## Project Structure

```
two_tank_system_sim/
├── README.md
├── task_1/
│   ├── TwoTankModel.slx
│   └── run_model.m
├── task_2/
│   ├── create_linearized_models.m
│   ├── build_ss_model.m
│   ├── build_tf_model.m
│   └── run_model_comparison.m
└── docs/
    ├── subject_predictive_control.pdf
    ├── task_1/
    │   ├── tank_levels.png
    │   └── flows.png
    └── task_2/
        └── task_2_comp.png
```

---

## Overview

This MATLAB/Simulink project simulates a two-tank water system for a **Predictive Control** semestral project. The system consists of two cylindrical tanks connected in series, where the goal is to learn system modelling, simulation, compare MCP and cascade PI control.

---

## System Description

### Physical System

- **Tank 1 (Upper)**: Receives inflow $Q$ from pump
- **Tank 2 (Lower)**: Receives flow $Q_{12}$ from Tank 1, outputs flow $Q_2$

Water flows through connecting pipes with smaller cross-sectional areas than the tanks.

---

## Task 1: Physical-Mathematical Model and Steady-State Analysis

### Objective

Create the nonlinear physical model and find the steady-state operating point of the system.

### Mathematical Modeling

The nonlinear model is derived from two fundamental principles:

#### Mass Balance Equation

For each tank, the rate of change of water volume equals the difference between inflow and outflow:

$A_1 \cdot \frac{dh_1}{dt} = Q_{in} - Q_{out}$

$A_2 \cdot \frac{dh_2}{dt} = Q_{in} - Q_{out}$

Where $A$ is the cross-sectional area of the tank.

#### Torricelli's Law

Water outflow through an orifice is proportional to the square root of the water height above the orifice:

$Q = a \cdot \sqrt{2gh}$

Where:
- $a$ = pipe cross-sectional area $[m^2]$
- $g$ = gravitational acceleration $[9.81 \, m/s^2]$
- $h$ = water height $[m]$

#### System Equations

Combining both principles, the complete nonlinear model for the two-tank system is:

$A_1 \cdot \frac{dh_1}{dt} = Q - a_1\sqrt{2g \cdot h_1}$

$A_2 \cdot \frac{dh_2}{dt} = a_1\sqrt{2g \cdot h_1} - a_2\sqrt{2g \cdot h_2}$

- Tank 1: inflow $Q$ from pump, outflow through pipe $a_1$ to Tank 2
- Tank 2: inflow from Tank 1 ($a_1\sqrt{2gh_1}$), outflow through pipe $a_2$

### Implementation

- **Simulink Model**: `task_1/TwoTankModel.slx`
- **Run Script**: `task_1/run_model.m`

The nonlinear model implements these equations for both tanks. The simulation runs for 10000 s.

### Steady-State Results

| Parameter | Value | Unit |
|-----------|-------|------|
| Water level $h_1$ | 0.816 | $[m]$ |
| Water level $h_2$ | 0.816 | $[m]$ |
| Flow rate $Q$ (inflow) | 0.01 | $[m^3/s]$ |
| Flow rate $Q_{12}$ (inter-tank) | 0.01 | $[m^3/s]$ |

**Note**: At steady-state, both tanks reach the same level (0.816 m) because $Q = Q_{12}$ when the system is at equilibrium.

### Results

Both tanks reach steady-state at 0.816 m after approximately 3000 seconds. The flow rates stabilize at $0.01 \, [m^3/s]$.

![Tank Levels](docs/task_1/tank_levels.png)

![Flow Rates](docs/task_1/flows.png)

---

## Task 2: Linearization in Deviation Form

### Objective

Linearize the nonlinear model around the steady-state operating point and implement it using both state-space and transfer function representations. Compare all three models (nonlinear, linear SS, linear TF) with different input step sizes.

### Linearized Model

The system is linearized around the steady-state ($h_{1s} = h_{2s} = 0.816 \, [m]$, $Q_s = 0.01 \, [m^3/s]$). The linearized equations in deviation form are:

$\frac{d\Delta h_1}{dt} = -\frac{g \cdot a_1}{A_1 \cdot \sqrt{h_1}} \cdot \Delta h_1 + \frac{1}{A_1} \cdot \Delta Q$

$\frac{d\Delta h_2}{dt} = \frac{g \cdot a_1}{A_2 \cdot \sqrt{h_1}} \cdot \Delta h_1 - \frac{g \cdot a_2}{A_2 \cdot \sqrt{h_2}} \cdot \Delta h_2$

### Implementation

| File | Description |
|------|-------------|
| `task_2/create_linearized_models.m` | Main script that computes linearization coefficients and builds both models |
| `task_2/build_ss_model.m` | Creates State-Space Simulink model (LinearSS.slx) |
| `task_2/build_tf_model.m` | Creates Transfer Function Simulink model (LinearTF.slx) |
| `task_2/run_model_comparison.m` | Compares all three models with different $Q_{in}$ step sizes (+10%, +30%, +50%) |

### Model Comparison

The comparison script simulates a step input in $Q_{in}$ (the inflow rate) with three different magnitudes:
- **+10%** of steady-state flow ($\Delta Q = 0.001 \, [m^3/s]$)
- **+30%** of steady-state flow ($\Delta Q = 0.003 \, [m^3/s]$)
- **+50%** of steady-state flow ($\Delta Q = 0.005 \, [m^3/s]$)

For each step size, the script compares:
1. **Nonlinear model** (original TwoTankModel.slx) - outputs absolute $h_2$
2. **Linear State-Space model** (LinearSS.slx) - outputs deviation $\Delta h_2$, converted to absolute
3. **Linear Transfer Function model** (LinearTF.slx) - outputs deviation $\Delta h_2$, converted to absolute

The results show how well the linear approximations match the nonlinear model for small and larger disturbances.

### State-Space Representation

State-space form: $\mathbf{x}(k+1) = \mathbf{A} \cdot \mathbf{x}(k) + \mathbf{B} \cdot \mathbf{u}(k) + \mathbf{G} \cdot \mathbf{d}(k)$

Output equation: $\mathbf{y} = \mathbf{C} \cdot \mathbf{x} + \mathbf{D} \cdot \mathbf{u}$

Where the output is $h_2$ (water level in Tank 2).

### Results

![Model Comparison](docs/task_2/task_2_comp.png)

The plot shows the response of $h_2$ for all three step sizes (+10%, +30%, +50%), demonstrating how well the linear approximations match the nonlinear model across different disturbance magnitudes.

---

## Next Steps (Future Tasks)

- Valves
- Cascade PI regulation
- MPC regulation
- Noise imitation on input
- MPC and cascade PI final comparison

