# Two-Tank Water System Simulation

## Overview

This MATLAB/Simulink model simulates a two-tank water system to find steady-state values for water levels and flow rates.

## Model Description

The system consists of two interconnected tanks with water flowing between them:
- **Tank 1**: Upper tank receiving inflow Q
- **Tank 2**: Lower tank receiving flow Q12 from Tank 1

### Steady-State Results

| Parameter | Value | Unit |
|-----------|-------|------|
| Water level h₁ | 0.816 | m |
| Water level h₂ | 0.816 | m |
| Flow rate Q (inflow) | 0.01 | m³/s |
| Flow rate Q₁₂ (inter-tank) | 0.01 | m³/s |

## Files

| File | Description |
|------|-------------|
| `TwoTankModel.slx` | Simulink model |
| `run_model.m` | MATLAB script to run simulation and generate plots |
| `docs/tank_levels.png` | Water levels over time |
| `docs/flows.png` | Flow rates over time |

## Running the Model

1. Open MATLAB
2. Run the simulation script:
   ```matlab
   run_model
   ```
3. Plots are saved to `docs/` directory

## Results

### Tank Levels
![Tank Levels](docs/tank_levels.png)

Both tanks reach steady-state at 0.816 m after approximately 3000 seconds.

### Flow Rates
![Flow Rates](docs/flows.png)

Both flow rates stabilize at 0.01 m³/s (10⁻² m³/s).