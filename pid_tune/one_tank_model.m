%% # Return Value:
%% 0 - success
%% 1 - error
%% 4 - model exists
function one_tank_model(mdl)
	mdl = char(mdl);
	if bdIsLoaded(mdl), close_system(mdl, 0); end
	new_system(mdl);
	run('../task_2/params.m'); 
	inputBlk = '/q_in';
	odeBlk = '/ODE';
	add_ode(mdl, odeBlk);
	add_block('simulink/Sources/Constant',        [mdl inputBlk]);
	add_block('simulink/Continuous/Integrator',   [mdl '/h']);
	set_param([mdl inputBlk],  'Value',              num2str(Q_ss));
	set_param([mdl '/h'],  'InitialCondition',   num2str(h_ss));
	add_valve(mdl, 'Valve', 0, 1, tau, z_ss);
	slave_gains = autotune(tau);
	master_gains = autotune(A);
	add_regulator(mdl, Q_ss, slave_gains, master_gains);

	add_block('simulink/Sources/Step', [mdl '/h_ref']);
	set_param([mdl '/h_ref'], ...
	'Time',         '500',          ...   % step at t=500s
	'Before', num2str(h_ss),  ...   % start at SS
	'After',   num2str(1.2 * h_ss)); % +20% reference step

	% connections
	add_line(mdl, [inputBlk(2:end) '/1'],  'ODE/3', 'autorouting','on');
	add_line(mdl, 'h/1',  'ODE/1', 'autorouting','on');
	add_line(mdl, 'ODE/1', 'h/1',  'autorouting','on');
	add_line(mdl, 'Sat/1', 'ODE/2', 'autorouting', 'on');
	add_line(mdl, 'Slave/1', 'Valve/1', 'autorouting', 'on');
	add_block('simulink/Math Operations/Sum', [mdl '/q_sum'], 'Inputs', '+-');
	add_line(mdl, 'Master/1', 'q_sum/1', 'autorouting','on');
	add_line(mdl, 'ODE/2', 'q_sum/2', 'autorouting','on');
	add_line(mdl, 'q_sum/1', 'Slave/1', 'autorouting','on');
	add_block('simulink/Math Operations/Sum', [mdl '/h_sum'], 'Inputs', '+-');
	add_line(mdl, 'h_ref/1', 'h_sum/1', 'autorouting','on');
	add_line(mdl, 'h/1', 'h_sum/2', 'autorouting','on');
	add_line(mdl, 'h_sum/1', 'Master/1', 'autorouting','on');

	% loggers
	add_block('simulink/Sinks/To Workspace', [mdl '/Log_q_out']);
	add_block('simulink/Sinks/To Workspace', [mdl '/Log_h']);
	add_block('simulink/Sinks/To Workspace', [mdl '/Log_z']);
	set_param([mdl '/Log_h'], 'VariableName', 'h_out', 'SaveFormat', 'Array');
	set_param([mdl '/Log_q_out'], 'VariableName', 'q_out', 'SaveFormat', 'Array');
	set_param([mdl '/Log_z'], 'VariableName', 'z_out', 'SaveFormat', 'Array');
	add_line(mdl, 'h/1', 'Log_h/1', 'autorouting','on');
	add_line(mdl, 'ODE/2', 'Log_q_out/1', 'autorouting','on');
	add_line(mdl, 'Sat/1', 'Log_z/1', 'autorouting','on');
	set_param(mdl, 'StopTime', '5000', 'Solver', 'ode45');

    Simulink.BlockDiagram.arrangeSystem(mdl);
	save_system(name);      
end

function add_ode(mdl, name)
	add_block('simulink/User-Defined Functions/MATLAB Function', [mdl name]);
	wrapperCode = [...
		'function dh, q_out = fcn(h, z, q_in)'  newline ...
		'[dh, q_out] = one_tank_ode(h, z, q_in);' newline ...
		'end'];
	addpath('../task_3');
	set_mfunction_block(mdl, name(2:end), wrapperCode);
end

function add_valve(mdl, name, min_limit, max_limit, tau_v, z_ss)
	add_block('simulink/Continuous/State-Space', [mdl '/' name]);
	set_param([mdl '/' name], ...
		'A',  num2str(-1/tau_v), ...   
		'B',  num2str(1/tau_v),  ...    
		'C',  '1', ...
		'D',  '0', ...
		'X0', num2str(z_ss));            

	add_block('simulink/Discontinuities/Saturation', [mdl '/Sat']);
	set_param([mdl '/Sat'],	...
		'UpperLimit', num2str(max_limit),	...
		'LowerLimit', num2str(min_limit));

	add_line(mdl, [name '/1'], 'Sat/1',   'autorouting', 'on');
end

function	add_regulator(mdl, Q_ss, slave_gains, master_gains)
	MasterBlk = [mdl '/Master'];
	SlaveBlk = [mdl '/Slave'];
	TuneMasterBlk   = [mdl '/TuneMaster'];
	TuneSlaveBlk   = [mdl '/TuneMaster'];
	add_block('simulink/Continuous/PID Controller', SlaveBlk);
	set_param(SlaveBlk,                                 ...
		'Controller',                   'PI',                    ...
		'AntiWindupMode',               'clamping',              ...
		'P',                            num2str(slave_gains.Kp),          ...
		'I',                            num2str(slave_gains.Ki),          ...
		'InitialConditionForIntegrator', num2str(0),    ...
		'LimitOutput',                  'on',                    ...
		'UpperSaturationLimit',         '1',                     ...  % max valve position
		'LowerSaturationLimit',         '0');
	add_block('simulink/Continuous/PID Controller', MasterBlk);
	set_param(MasterBlk,                                 ...
		'Controller',                   'PI',                    ...
		'P',                            num2str(master_gains.Kp),          ...
		'I',                            num2str(master_gains.Ki),          ...
		'AntiWindupMode',               'clamping',              ...
		'InitialConditionForIntegrator', num2str(0),    ...
		'LimitOutput',                  'on',	...
		'UpperSaturationLimit',         num2str(Q_ss * 2),	...  % max valve position
		'LowerSaturationLimit',         '0');
end

function	gains = autotune(var)
	transfer = tf(1, [var, 1]);
	gains = pidtune(transfer, 'PI');
end

