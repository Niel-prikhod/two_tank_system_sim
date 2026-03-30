%% two_tank_simulink.m
% Programmatically creates the two-tank Simulink model.
% Requires Simulink. Run: two_tank_simulink; open_system('TwoTankModel')

%% Parameters
A   = 4;        % m²  — tank cross-section
S   = 0.0025;   % m²  — pipe cross-section
C   = 1;        % —    discharge coefficient
g   = 9.81;     % m/s²
Q1  = 0.01;     % m³/s — inlet flow (constant)
Q2_ss = 0.01;   % m³/s — nominal outlet flow

% Steady-state ICs (derived above)
dh_ss = Q1^2 / (C^2 * S^2 * 2 * g);   % 0.8155 m
h2_ss = dh_ss;                          % 0.8155 m
h1_ss = 2 * dh_ss;                      % 1.631  m

%% Create / reset model
mdl = 'TwoTankModel';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'StopTime', '3000');     % 50-minute simulation

%% Helper: add and position block
addBlk = @(lib, name, pos) add_block(lib, [mdl '/' name], 'Position', pos);

%% ── SOURCES ──────────────────────────────────────────────────────────────
addBlk('simulink/Sources/Constant',          'Q1',  [30 100 80 130]);
set_param([mdl '/Q1'], 'Value', num2str(Q1));

addBlk('simulink/Sources/Step',              'Q2',  [30 300 80 330]);
set_param([mdl '/Q2'], 'Time',         '500', ...
                        'InitialValue', num2str(Q2_ss), ...
                        'FinalValue',   '0.012');   % example step +20%

%% ── TANK 1 LOOP ──────────────────────────────────────────────────────────
addBlk('simulink/Math Operations/Sum',       'Sum1', [160  100 200 140]);
set_param([mdl '/Sum1'], 'Inputs', '+-');

addBlk('simulink/Math Operations/Gain',      'invA1', [240 100 300 130]);
set_param([mdl '/invA1'], 'Gain', num2str(1/A));

addBlk('simulink/Continuous/Integrator',     'h1',   [360 90 420 140]);
set_param([mdl '/h1'], 'InitialCondition', num2str(h1_ss));

%% ── TANK 2 LOOP ──────────────────────────────────────────────────────────
addBlk('simulink/Math Operations/Sum',       'Sum2', [160 300 200 340]);
set_param([mdl '/Sum2'], 'Inputs', '+-');

addBlk('simulink/Math Operations/Gain',      'invA2', [240 300 300 330]);
set_param([mdl '/invA2'], 'Gain', num2str(1/A));

addBlk('simulink/Continuous/Integrator',     'h2',   [360 290 420 340]);
set_param([mdl '/h2'], 'InitialCondition', num2str(h2_ss));

%% ── NONLINEAR PIPE FLOW SUBSYSTEM ────────────────────────────────────────
addBlk('simulink/Math Operations/Sum',          'dh',     [160 200 200 240]);
set_param([mdl '/dh'],   'Inputs', '+-');

addBlk('simulink/Math Operations/Abs',          'AbsDh',  [230 200 270 240]);
addBlk('simulink/Math Operations/Gain',         'Gain2g', [300 200 350 240]);
set_param([mdl '/Gain2g'], 'Gain', num2str(2*g));

addBlk('simulink/Math Operations/Math Function','Sqrt',   [380 200 430 240]);
set_param([mdl '/Sqrt'], 'Operator', 'sqrt');

addBlk('simulink/Math Operations/Gain',         'GainCS', [460 200 510 240]);
set_param([mdl '/GainCS'], 'Gain', num2str(C*S));

addBlk('simulink/Math Operations/Sign',         'Sgn',    [230 260 270 290]);
addBlk('simulink/Math Operations/Product',      'Q12',    [540 195 580 245]);

%% ── SCOPE ────────────────────────────────────────────────────────────────
addBlk('simulink/Sinks/Scope', 'Scope', [500 100 550 150]);

%% ── WIRING ───────────────────────────────────────────────────────────────
connect = @(src, dst) add_line(mdl, src, dst, 'autorouting', 'on');

% Sources → Sum1
connect('Q1/1',    'Sum1/1');
connect('Q12/1',   'Sum1/2');   % pipe flow subtracted from tank 1

% Sum1 → 1/A → Integrator h1
connect('Sum1/1',  'invA1/1');
connect('invA1/1', 'h1/1');

% h1, h2 → Δh
connect('h1/1',    'dh/1');
connect('h2/1',    'dh/2');

% Δh → |Δh| → 2g*|Δh| → sqrt → C*S  (magnitude path)
connect('dh/1',    'AbsDh/1');
connect('AbsDh/1', 'Gain2g/1');
connect('Gain2g/1','Sqrt/1');
connect('Sqrt/1',  'GainCS/1');

% Δh → sign  (direction path)
connect('dh/1',    'Sgn/1');

% sign * magnitude → Q12
connect('GainCS/1','Q12/1');
connect('Sgn/1',   'Q12/2');

% Q12, Q2 → Sum2
connect('Q12/1',   'Sum2/1');
connect('Q2/1',    'Sum2/2');

% Sum2 → 1/A → Integrator h2
connect('Sum2/1',  'invA2/1');
connect('invA2/1', 'h2/1');

% h2 → Scope
connect('h2/1',    'Scope/1');

%% Done
save_system(mdl);
disp('Model built. Run simulation or connect your MPC Controller block to Q2.');
