run('../task_2/params.m');   % loads A, a, g, h_ss, Q_ss, k

tau_v = 10;        % s  — valve time constant
z_ss  = 0.5;       % nominal valve opening at operating point
Kv    = Q_ss / (z_ss * sqrt(2 * g * h_ss));   % = 0.005 m²

%% Build model
mdl = 'model_w_valves';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

% Blocks
add_block('simulink/Sources/Constant',        [mdl '/Q1'],  'Position', [30  140 80  160]);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
                                               [mdl '/ODE'], 'Position', [160 120 280 180]);
wrapperCode = [...
    'function [dh1, dh2] = step(Q_in, h1, h2, z1, z2)'  newline ...
    '    [dh1, dh2] = two_tank_ode(h1, h2, Q_in, z1, z2);' newline ...
    'end'];

set_mfunction_block(mdl, 'ODE', wrapperCode);

add_block('simulink/Continuous/Integrator',   [mdl '/h1'],  'Position', [340 100 390 140]);
add_block('simulink/Continuous/Integrator',   [mdl '/h2'],  'Position', [340 160 390 200]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log_h1'], 'Position', [460  80 540 100]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log_h2'], 'Position', [460 200 540 220]);

% Parameters
set_param([mdl '/Q1'],  'Value',              num2str(Q_ss));
set_param([mdl '/h1'],  'InitialCondition',   num2str(h_ss));
set_param([mdl '/h2'],  'InitialCondition',   num2str(h_ss));

set_param([mdl '/Log_h1'], 'VariableName', 'h1_out', 'SaveFormat', 'Array');
set_param([mdl '/Log_h2'], 'VariableName', 'h2_out', 'SaveFormat', 'Array');

% Connect h1 and h2 outputs to their respective loggers


% Model settings
set_param(mdl, 'StopTime', '3000', 'Solver', 'ode45');

% Connections
add_line(mdl, 'Q1/1',  'ODE/1', 'autorouting','on');
add_line(mdl, 'h1/1',  'ODE/2', 'autorouting','on');
add_line(mdl, 'h2/1',  'ODE/3', 'autorouting','on');
add_line(mdl, 'ODE/1', 'h1/1',  'autorouting','on');
add_line(mdl, 'ODE/2', 'h2/1',  'autorouting','on');
add_line(mdl, 'h1/1', 'Log_h1/1', 'autorouting','on');
add_line(mdl, 'h2/1', 'Log_h2/1', 'autorouting','on');

%% add_valves.m
% Extends the copied nonlinear model with two control valves.
% Assumes: TwoTankValveModel.slx already contains Q_in source,
%          two Integrators (h1, h2), MATLAB Function block (ODE),
%          and To Workspace loggers.


%% ── VALVE 1 subsystem (controls Q12) ────────────────────────────────────
% Step 1a: Command signal — z1_cmd
% Starts at z_ss (steady state), steps to test open/close behaviour
add_block('simulink/Sources/Step', [mdl '/z1_cmd'], ...
    'Position', [30 220 80 250]);
set_param([mdl '/z1_cmd'], ...
    'Time',         '500', ...       % step at t=500s
    'Before', num2str(z_ss), ...
    'After',   '0.8');          % open further — change to test closing

% Step 1b: First-order valve dynamics — 1/(tau_v * s + 1)
add_block('simulink/Continuous/State-Space', [mdl '/Valve1']);
set_param([mdl '/Valve1'], ...
    'A',  num2str(-1/tau_v), ...     % -0.1
    'B',  num2str(1/tau_v),  ...     %  0.1
    'C',  '1', ...
    'D',  '0', ...
    'X0', num2str(z_ss));            %  0.5  — initial valve position

% Step 1c: Saturation — clamp actual position to [0, 1]
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat1'], ...
    'Position', [260 215 310 255]);
set_param([mdl '/Sat1'], 'UpperLimit', '1', 'LowerLimit', '0');

% Wire valve 1 chain
add_line(mdl, 'z1_cmd/1', 'Valve1/1', 'autorouting', 'on');
add_line(mdl, 'Valve1/1', 'Sat1/1',   'autorouting', 'on');

%% ── VALVE 2 subsystem (controls Q_out) ──────────────────────────────────
add_block('simulink/Sources/Step', [mdl '/z2_cmd'], ...
    'Position', [30 300 80 330]);
set_param([mdl '/z2_cmd'], ...
    'Time',         '500', ...
    'Before', num2str(z_ss), ...
    'After',   '0.2');          % partially close — change to test

add_block('simulink/Continuous/State-Space', [mdl '/Valve2']);
set_param([mdl '/Valve2'], ...
    'A',  num2str(-1/tau_v), ...     % -0.1
    'B',  num2str(1/tau_v),  ...     %  0.1
    'C',  '1', ...
    'D',  '0', ...
    'X0', num2str(z_ss));            %  0.5  — initial valve position

add_block('simulink/Discontinuities/Saturation', [mdl '/Sat2'], ...
    'Position', [260 295 310 335]);
set_param([mdl '/Sat2'], 'UpperLimit', '1', 'LowerLimit', '0');

add_line(mdl, 'z2_cmd/1', 'Valve2/1', 'autorouting', 'on');
add_line(mdl, 'Valve2/1', 'Sat2/1',   'autorouting', 'on');

%% ── Connect valve outputs to ODE block ───────────────────────────────────
% The MATLAB Function block (ODE) now has 5 inputs:
%   1: Q_in,  2: h1,  3: h2,  4: z1,  5: z2
% Ports 4 and 5 are new — connect saturation outputs to them
add_line(mdl, 'Sat1/1', 'ODE/4', 'autorouting', 'on');
add_line(mdl, 'Sat2/1', 'ODE/5', 'autorouting', 'on');

%% ── Log valve positions ──────────────────────────────────────────────────
add_block('simulink/Sinks/To Workspace', [mdl '/Log_z1'], ...
    'Position', [380 215 460 245]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log_z2'], ...
    'Position', [380 295 460 325]);

set_param([mdl '/Log_z1'], 'VariableName', 'z1_out', 'SaveFormat', 'Array');
set_param([mdl '/Log_z2'], 'VariableName', 'z2_out', 'SaveFormat', 'Array');

add_line(mdl, 'Sat1/1', 'Log_z1/1', 'autorouting', 'on');
add_line(mdl, 'Sat2/1', 'Log_z2/1', 'autorouting', 'on');

%% ── Model settings ───────────────────────────────────────────────────────
set_param(mdl, 'StopTime', '3000', 'Solver', 'ode45');
save_system(mdl);
disp('Valves added. Update ODE block wrapper to call two_tank_ode with z1, z2.');
