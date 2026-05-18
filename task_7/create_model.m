function mdl = create_model(controller)
	run('../task_2/params.m');

	tau_v = 10;
	z_ss  = 0.5;
	Kv    = Q_ss / (z_ss * sqrt(2 * g * h_ss));   % = 0.005 m²

	max_time = 10000;

	%% Build model
	mdl = [controller '_model'];
	if bdIsLoaded(mdl), close_system(mdl, 0); end
	new_system(mdl); open_system(mdl);

	% Blocks
	add_block('simulink/Sources/From Workspace', ...
		[mdl '/Q1']);
	add_block('simulink/User-Defined Functions/MATLAB Function', ...
		[mdl '/ODE']);
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
	set_param([mdl '/Q1'],	'VariableName', 'q_in');
	set_param([mdl '/Q1'],	'SampleTime',	'-1');
	set_param([mdl '/h1'],  'InitialCondition',   '0');
	set_param([mdl '/h2'],  'InitialCondition',   '0');

	set_param([mdl '/Log_h1'], 'VariableName', 'h1_out', 'SaveFormat', 'Array');
	set_param([mdl '/Log_h2'], 'VariableName', 'h2_out', 'SaveFormat', 'Array');

	% Model settings
	set_param(mdl, 'StopTime', num2str(max_time), 'Solver', 'ode45');

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

	set_param(mdl, 'StopTime', '3000', 'Solver', 'ode45');

	% Autotune PI gains

	Kv_lin  = Kv * sqrt(2 * g * h_ss);          % linearised gain ~0.02 m³/s
	G_slave = tf(Kv_lin, [tau_v, 1]);            % 0.02 / (10s + 1)
	C_slave = pidtune(G_slave, 'PI');
	CL_slave  = feedback(C_slave * G_slave, 1);
	Kp_s = C_slave.Kp;    Ki_s = C_slave.Ki;

if strcmp(controller, 'pid')
	[Kp_m, Ki_m] = tune_master(A, CL_slave);
end

	h1_ref = set_reference();
	time_offset = 2000;
	h2_ref = add_offset(h1_ref, time_offset);
	save('reference.mat', 'h1_ref', 'h2_ref');

	add_block('simulink/Sources/From Workspace', [mdl '/h1_ref']);
	set_param([mdl '/h1_ref'], 'VariableName', 'h1_ref'); 
	set_param([mdl '/h1_ref'], 'SampleTime', '-1'); 
	add_block('simulink/Sources/From Workspace', [mdl '/h2_ref']);
	set_param([mdl '/h2_ref'], 'VariableName', 'h2_ref'); 
	set_param([mdl '/h2_ref'], 'SampleTime', '-1'); 

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


	% Error convention: e_slave = Q12_ref - Q12_meas
	% Cascade loop 1 (Tank 1: controls h1 via z1)
	add_block('simulink/Math Operations/Sum', [mdl '/SlaveErr1']);
	set_param([mdl '/SlaveErr1'], 'Inputs', '+-');
	add_block('simulink/Continuous/PID Controller', [mdl '/SlavePI1']);
	set_param([mdl '/SlavePI1'],                                 ...
		'Controller',                   'PI',                    ...
		'P',                            num2str(Kp_s),           ...
		'I',                            num2str(Ki_s),           ...
		'AntiWindupMode',               'clamping',              ...
		'InitialConditionForIntegrator', '0',				    ...
		'LimitOutput',                  'on',                    ...
		'UpperSaturationLimit',         '1',                     ...  % max valve position
		'LowerSaturationLimit',         '0');
	add_line(mdl, 'FlowMeas1/1', 'SlaveErr1/2',  'autorouting', 'on'); % Q12_meas→ -
	add_line(mdl, 'SlaveErr1/1', 'SlavePI1/1',   'autorouting', 'on'); % error   → slave PI
	add_line(mdl, 'SlavePI1/1',  'Valve1/1',     'autorouting', 'on'); % z1_cmd  → valve

	% Cascade loop 2
	add_block('simulink/Math Operations/Sum', [mdl '/SlaveErr2']);
	set_param([mdl '/SlaveErr2'], 'Inputs', '+-');
	add_block('simulink/Continuous/PID Controller', [mdl '/SlavePI2']);
	set_param([mdl '/SlavePI2'],                             ...
		'Controller',                   'PI',                    ...
		'P',                            num2str(Kp_s),           ...
		'I',                            num2str(Ki_s),           ...
		'AntiWindupMode',               'clamping',              ...
		'InitialConditionForIntegrator', '0',				    ...
		'LimitOutput',                  'on',                    ...
		'UpperSaturationLimit',         '1',                     ...
		'LowerSaturationLimit',         '0');
	add_line(mdl, 'FlowMeas2/1', 'SlaveErr2/2',  'autorouting', 'on');
	add_line(mdl, 'SlaveErr2/1', 'SlavePI2/1',   'autorouting', 'on');
	add_line(mdl, 'SlavePI2/1',  'Valve2/1',     'autorouting', 'on');

	if strcmp(controller, 'mpc')
		add_mpc(mdl, Kv, h_ss, A, g, Q_ss);
	elseif strcmp(controller, 'pid')
		add_pid(mdl, Q_ss, Kp_m, Ki_m);
	end

	save_system(mdl);
	close_system(mdl);
end
