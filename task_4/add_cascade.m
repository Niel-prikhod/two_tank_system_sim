run('../task_2/params.m');   % loads A, a, g, h_ss, Q_ss, k

tau_v = 10;        % s  — valve time constant
z_ss  = 0.5;       % nominal valve opening at operating point
Kv    = Q_ss / (z_ss * sqrt(2 * g * h_ss));   % = 0.005 m²

%% Build model
mdl = 'model_w_cascade';
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

addpath('../task_3');
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

%% ── VALVE 1 subsystem (controls Q12) ────────────────────────────────────

add_block('simulink/Continuous/State-Space', [mdl '/Valve1']);
set_param([mdl '/Valve1'], ...
'A',  num2str(-1/tau_v), ...     % -0.1
'B',  num2str(1/tau_v),  ...     %  0.1
'C',  '1', ...
'D',  '0', ...
'X0', num2str(z_ss));            %  0.5  — initial valve position

% Saturation — clamp actual position to [0, 1]
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat1'], ...
'Position', [260 215 310 255]);
set_param([mdl '/Sat1'], 'UpperLimit', '1', 'LowerLimit', '0');

add_line(mdl, 'Valve1/1', 'Sat1/1',   'autorouting', 'on');

%% ── VALVE 2 subsystem (controls Q_out) ──────────────────────────────────
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
disp('Valves added. Update ODE block wrapper to call two_tank_ode with z1, z2.');

%% ═══════════════════════════════════════════════════════════
%% Autotune PI gains
%% ═══════════════════════════════════════════════════════════

% Slave plant: z_cmd → Q_meas
% Valve is 1st order, flow is linear in z at operating point
Kv_lin  = Kv * sqrt(2 * g * h_ss);          % linearised gain ~0.02 m³/s
G_slave = tf(Kv_lin, [tau_v, 1]);            % 0.02 / (10s + 1)

% Tune slave PI against slave plant
C_slave = pidtune(G_slave, 'PI');

% Master plant: Q_ref → h
% With slave closed, inner loop is approximated as CL_slave
% Master sees: Q_ref → [CL_slave] → Q → [1/As] → h
CL_slave  = feedback(C_slave * G_slave, 1);
G_master  = tf(1, [A, 0]);                   % integrator 1/(As)
G_m_total = G_master * CL_slave;             % full master plant

% Tune master PI against full plant
C_master = pidtune(G_m_total, 'PI');

% Extract gains
Kp_s = C_slave.Kp;    Ki_s = C_slave.Ki;
Kp_m = C_master.Kp;   Ki_m = C_master.Ki;

fprintf('\n=== Autotuned PI gains ===\n');
fprintf('Slave  — Kp: %8.5f   Ki: %8.5f\n', Kp_s, Ki_s);
fprintf('Master — Kp: %8.5f   Ki: %8.5f\n', Kp_m, Ki_m);

%% ═══════════════════════════════════════════════════════════
%% Reference signal (shared by both tanks)
%% ═══════════════════════════════════════════════════════════

% Single Step block — same h_ref feeds both master loops
% Initial value = h_ss so simulation starts at operating point
add_block('simulink/Sources/Step', [mdl '/h_ref']);
set_param([mdl '/h_ref'], ...
'Time',         '500',          ...   % step at t=500s
'Before', num2str(h_ss),  ...   % start at SS
'After',   num2str(1.2 * h_ss)); % +20% reference step

% Log h_ref for plotting
add_block('simulink/Sinks/To Workspace', [mdl '/Log_href']);
set_param([mdl '/Log_href'], 'VariableName', 'href_out', 'SaveFormat', 'Array');
add_line(mdl, 'h_ref/1', 'Log_href/1', 'autorouting', 'on');

%% ═══════════════════════════════════════════════════════════
%% Flow measurement blocks
%% ═══════════════════════════════════════════════════════════

% Q_meas = Kv * z * sqrt(2g * h)
% Implemented as MATLAB Function block — two inputs (h, z), one output (Q)
flowCode = strjoin({
'function Q = measure_flow(h, z)'
'    Kv_v = 0.005;'
'    g_v  = 9.81;'
'    Q = Kv_v * z * sqrt(2 * g_v * max(h, 0));'
'end'
}, newline);

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/FlowMeas1']);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/FlowMeas2']);

set_mfunction_block(mdl, 'FlowMeas1', flowCode);
set_mfunction_block(mdl, 'FlowMeas2', flowCode);

% Inputs: h from integrator, z from saturation output (actual valve position)
add_line(mdl, 'h1/1',   'FlowMeas1/1', 'autorouting', 'on');
add_line(mdl, 'Sat1/1', 'FlowMeas1/2', 'autorouting', 'on');

add_line(mdl, 'h2/1',   'FlowMeas2/1', 'autorouting', 'on');
add_line(mdl, 'Sat2/1', 'FlowMeas2/2', 'autorouting', 'on');

%% ═══════════════════════════════════════════════════════════
%% Cascade loop 1 (Tank 1: controls h1 via z1)
%% ═══════════════════════════════════════════════════════════

% ── Master loop ──────────────────────────────────────────────
% Error convention: e_master = h1 - h_ref
% Positive error (h1 > h_ref) → increase Q12_ref → open z1 → h1 drops
add_block('simulink/Math Operations/Sum', [mdl '/MasterErr1']);
set_param([mdl '/MasterErr1'], 'Inputs', '+-');

add_block('simulink/Continuous/PID Controller', [mdl '/MasterPI1']);
set_param([mdl '/MasterPI1'],                               ...
'Controller',                   'PI',                   ...
'P',                            num2str(Kp_m),          ...
'I',                            num2str(Ki_m),          ...
'LimitOutput',                  'on',                   ...
'UpperSaturationLimit',         num2str(2 * Q_ss),      ...  % max Q12_ref
'LowerSaturationLimit',         '0');

% ── Slave loop ───────────────────────────────────────────────
% Error convention: e_slave = Q12_ref - Q12_meas
% Positive error → increase z1_cmd → valve opens → Q12 rises
add_block('simulink/Math Operations/Sum', [mdl '/SlaveErr1']);
set_param([mdl '/SlaveErr1'], 'Inputs', '+-');

add_block('simulink/Continuous/PID Controller', [mdl '/SlavePI1']);
set_param([mdl '/SlavePI1'],                                 ...
'Controller',                   'PI',                    ...
'P',                            num2str(Kp_s),           ...
'I',                            num2str(Ki_s),           ...
'LimitOutput',                  'on',                    ...
'UpperSaturationLimit',         '1',                     ...  % max valve position
'LowerSaturationLimit',         '0');

% ── Wire loop 1 ──────────────────────────────────────────────
add_line(mdl, 'h1/1',        'MasterErr1/1', 'autorouting', 'on'); % h1      → +
add_line(mdl, 'h_ref/1',     'MasterErr1/2', 'autorouting', 'on'); % h_ref   → -
add_line(mdl, 'MasterErr1/1','MasterPI1/1',  'autorouting', 'on'); % error   → master PI
add_line(mdl, 'MasterPI1/1', 'SlaveErr1/1',  'autorouting', 'on'); % Q12_ref → +
add_line(mdl, 'FlowMeas1/1', 'SlaveErr1/2',  'autorouting', 'on'); % Q12_meas→ -
add_line(mdl, 'SlaveErr1/1', 'SlavePI1/1',   'autorouting', 'on'); % error   → slave PI
add_line(mdl, 'SlavePI1/1',  'Valve1/1',     'autorouting', 'on'); % z1_cmd  → valve

%% ═══════════════════════════════════════════════════════════
%% Cascade loop 2 (Tank 2: controls h2 via z2)
%% ═══════════════════════════════════════════════════════════

% Identical structure to loop 1 — same tuning (equal tanks and orifices)

% ── Master loop ──────────────────────────────────────────────
add_block('simulink/Math Operations/Sum', [mdl '/MasterErr2']);
set_param([mdl '/MasterErr2'], 'Inputs', '+-');

add_block('simulink/Continuous/PID Controller', [mdl '/MasterPI2']);
set_param([mdl '/MasterPI2'],                                ...
'Controller',                   'PI',                    ...
'P',                            num2str(Kp_m),           ...
'I',                            num2str(Ki_m),           ...
'LimitOutput',                  'on',                    ...
'UpperSaturationLimit',         num2str(2 * Q_ss),       ...
'LowerSaturationLimit',         '0');

% ── Slave loop ───────────────────────────────────────────────
add_block('simulink/Math Operations/Sum', [mdl '/SlaveErr2']);
set_param([mdl '/SlaveErr2'], 'Inputs', '+-');

add_block('simulink/Continuous/PID Controller', [mdl '/SlavePI2']);
set_param([mdl '/SlavePI2'],                                 ...
'Controller',                   'PI',                    ...
'P',                            num2str(Kp_s),           ...
'I',                            num2str(Ki_s),           ...
'LimitOutput',                  'on',                    ...
'UpperSaturationLimit',         '1',                     ...
'LowerSaturationLimit',         '0');

% ── Wire loop 2 ──────────────────────────────────────────────
add_line(mdl, 'h2/1',        'MasterErr2/1', 'autorouting', 'on');
add_line(mdl, 'h_ref/1',     'MasterErr2/2', 'autorouting', 'on');
add_line(mdl, 'MasterErr2/1','MasterPI2/1',  'autorouting', 'on');
add_line(mdl, 'MasterPI2/1', 'SlaveErr2/1',  'autorouting', 'on');
add_line(mdl, 'FlowMeas2/1', 'SlaveErr2/2',  'autorouting', 'on');
add_line(mdl, 'SlaveErr2/1', 'SlavePI2/1',   'autorouting', 'on');
add_line(mdl, 'SlavePI2/1',  'Valve2/1',     'autorouting', 'on');

save_system(mdl);
disp('Cascade PI control added and model saved.');
