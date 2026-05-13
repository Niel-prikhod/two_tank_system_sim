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
├── task_3/
│   ├── model_w_valves.slx
│   ├── build_model_w_valves.m
│   ├── run_valve_test.m
│   ├── set_mfunction_block.m
│   └── two_tank_ode.m
└── docs/
    ├── subject_predictive_control.pdf
    ├── task_1/
    │   ├── tank_levels.png
    │   └── flows.png
    ├── task_2/
    │   └── task_2_comp.png
    └── task_3/
        └── valve_function.png
```

---

## Overview

This MATLAB/Simulink project simulates a two-tank water system for a **Predictive Control** semestral project. The system consists of two cylindrical tanks connected in series, where the goal is to learn system modelling, simulation, compare MPC and cascade PI control.

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

## Task 3: Control Valves with First-Order Dynamics

### Objective

Extend the nonlinear two-tank model from Task 1 by adding two control valves — one on the outlet of Tank 1 (controlling $Q_{12}$) and one on the outlet of Tank 2 (controlling $Q_{out}$).

### Mathematical Model

#### Valve Flow Equation

The fixed orifice flow equation from Task 1:

$Q = a \cdot \sqrt{2g \cdot h}$

was replaced by a controllable valve flow equation:

$Q = K_v \cdot z \cdot \sqrt{2g \cdot h}$

Where:
- $z \in [0,1]$ is the valve opening position ($0$ = fully closed, $1$ = fully open)
- $K_v$ is the valve flow factor $[m^2]$

#### Valve Dynamics

Each valve has first-order dynamics modelling the physical lag of the valve actuator. The commanded position $z_{cmd}$ does not take effect instantly — the actual position $z$ responds as:

$\tau_v \cdot \frac{dz}{dt} = z_{cmd} - z$

Which in transfer function form is:

$\frac{Z(s)}{Z_{cmd}(s)} = \frac{1}{\tau_v \cdot s + 1}$

In Simulink, this is implemented as a State-Space block with:
- $A = -1/\tau_v$
- $B = 1/\tau_v$
- $C = 1$, $D = 0$

Followed by a Saturation block clamping $z$ to $[0, 1]$.

### Constants

| Parameter | Value | Unit | Justification |
|-----------|-------|------|---------------|
| $K_v$ | 0.005 | $[m^2]$ | Chosen so nominal operating point ($h_{1ss} = h_{2ss} = 0.816 \, [m]$, $Q_{ss} = 0.01 \, [m^3/s]$) is maintained at $z_{ss} = 0.5$. Derived from: $K_v = Q_{ss} / (z_{ss} \cdot \sqrt{2 \cdot g \cdot h_{ss}}) = 0.01 / (0.5 \cdot 4.0) = 0.005 \, [m^2]$ |
| $\tau_v$ | 10 | $[s]$ | Chosen to be realistic for an industrial control valve while remaining fast relative to the tank time constant ($\tau_{tank} \approx 652 \, [s]$). The valve fully settles in approximately $3\tau_v = 30 \, [s]$, which is less than 5% of the dominant system time constant. |
| $z_{ss}$ | 0.5 | $[-]$ | Nominal steady-state opening for both valves. Symmetric choice ensures equal headroom for opening and closing. |

### Implementation

- **Simulink Model**: `task_3/model_w_valves.slx`
- **Build Script**: `task_3/build_model_w_valves.m`
- **Run Script**: `task_3/run_valve_test.m`

### Test Description

Both valves start at $z_{ss} = 0.5$ and the system starts at the steady-state operating point. At $t = 500 \, [s]$, both valves receive a simultaneous step command:
- **Valve 1** ($z_1$): opens from 0.5 to 0.8 — increases $Q_{12}$
- **Valve 2** ($z_2$): closes from 0.5 to 0.3 — reduces $Q_{out}$

The actual valve positions lag behind the step commands due to the first-order dynamics with $\tau_v = 10 \, [s]$.

### Results

$h_1$ decreases after $t = 500 \, [s]$ because Valve 1 opens and drains Tank 1 faster than $Q_{in}$ replenishes it.

$h_2$ initially rises because Valve 1 increases inflow to Tank 2 while Valve 2 simultaneously restricts outflow. After the transient, $h_2$ settles at a new higher steady state.

The valve position plots show the first-order lag — actual $z$ tracks $z_{cmd}$ with a smooth exponential approach rather than an instantaneous jump.

The saturation block has no visible effect in this test since neither valve command exceeds $[0, 1]$, but it protects the model during edge-case inputs.

![Valve Test Results](docs/task_3/valve_function.png "Tank levels and valve positions over time")

---

## Next Steps (Future Tasks)

- Cascade PI regulation
- MPC regulation
- Noise imitation on input
- MPC and cascade PI final comparison

