function add_mpc(mdl, Kv, h_ss, A, g, Q_ss)
	Kv_lin = Kv * sqrt(2 * g * h_ss);   % max slave flow ~0.02 m³/s
	A_p = zeros(2);
	B_p = [-1/A,  0,    1/A ;   % columns: Q12_ref, Qout_ref, Q_in
		1/A, -1/A,  0   ];
	C_p = eye(2);
	D_p = zeros(2, 3);
	plant_c = ss(A_p, B_p, C_p, D_p);
	plant_c = setmpcsignals(plant_c, 'MV', [1 2], 'MD', 3);

	Ts_mpc  = 1;
	plant_d = c2d(plant_c, Ts_mpc, 'zoh');
	p = 20;   % prediction horizon: 20 × 30 s = 600 s ≈ 1 tank time constant
	m = 3;    % control horizon: 3 free moves per window
	mpcobj = mpc(plant_d, Ts_mpc, p, m);
	mpcobj.Weights.OutputVariables          = [10,   10  ];  % h1, h2 tracking
	mpcobj.Weights.ManipulatedVariables     = [0.01, 0.01];  % allow MVs to move freely
	mpcobj.Weights.ManipulatedVariablesRate = [0.1,  0.1 ];  % penalise large MV jumps
	mpcobj.MV(1).Min = 0;   mpcobj.MV(1).Max = Kv_lin;   % Q12_ref  ∈ [0, 0.02]
	mpcobj.MV(2).Min = 0;   mpcobj.MV(2).Max = Kv_lin;   % Qout_ref ∈ [0, 0.02]
	mpcobj.Model.Nominal.X  = [h_ss;  h_ss];
	mpcobj.Model.Nominal.U  = [Q_ss;  Q_ss;  Q_ss];   % [Q12_ref; Qout_ref; Q_in]
	mpcobj.Model.Nominal.Y  = [h_ss;  h_ss];
	mpcobj.Model.Nominal.DX = [0;     0   ];
	save('mpc_object.mat', 'mpcobj');

	add_block('mpclib/MPC Controller', [mdl '/MPC_Master']);
	set_param([mdl '/MPC_Master'], 'MpcObj', 'mpcobj');
	add_block('simulink/Signal Routing/Mux', [mdl '/MeasMux']);
	set_param([mdl '/MeasMux'], 'Inputs', '2');
	add_line(mdl, 'h1/1',      'MeasMux/1',    'autorouting', 'on');
	add_line(mdl, 'h2/1',      'MeasMux/2',    'autorouting', 'on');
	add_line(mdl, 'MeasMux/1', 'MPC_Master/1', 'autorouting', 'on');
	add_block('simulink/Signal Routing/Mux', [mdl '/RefMux']);
	set_param([mdl '/RefMux'], 'Inputs', '2');
	add_line(mdl, 'h1_ref/1',  'RefMux/1',     'autorouting', 'on');
	add_line(mdl, 'h2_ref/1',  'RefMux/2',     'autorouting', 'on');
	add_line(mdl, 'RefMux/1', 'MPC_Master/2', 'autorouting', 'on');
	add_line(mdl, 'Q1/1', 'MPC_Master/3', 'autorouting', 'on');
	add_block('simulink/Signal Routing/Demux', [mdl '/MVDemux']);
	set_param([mdl '/MVDemux'], 'Outputs', '2');
	add_line(mdl, 'MPC_Master/1', 'MVDemux/1',   'autorouting', 'on');
	add_line(mdl, 'MVDemux/1',    'SlaveErr1/1', 'autorouting', 'on');
	add_line(mdl, 'MVDemux/2',    'SlaveErr2/1', 'autorouting', 'on');
end
