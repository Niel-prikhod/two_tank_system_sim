function build_ss_model(k, A)

	% Construct matrix:
	A_mat = [-k/A, 0;
			k/A, -k/A];
	B_mat = [1/A;
				0];
	C_mat = [1, 0; 
        0, 1];
	D_mat = [0; 0];

    mdl = 'LinearSS';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    % Input: Delta_Q step
    add_block('simulink/Sources/Step',            [mdl '/DeltaQ']);
    set_param([mdl '/DeltaQ'], ...
        'Time',         '200', ...
        'Before', '0',   ...
        'After',   '0.001');   % +10% placeholder; overridden at runtime

    % State-Space block
    add_block('simulink/Continuous/State-Space',  [mdl '/SS']);
    set_param([mdl '/SS'], ...
        'A', mat2str(A_mat), ...
        'B', mat2str(B_mat), ...
        'C', mat2str(C_mat), ...
        'D', mat2str(D_mat), ...
        'X0', '[0; 0]');           % start at deviation = 0

    % To Workspace blocks for both outputs
    add_block('simulink/Sinks/To Workspace',      [mdl '/h2']);
    add_block('simulink/Sinks/To Workspace',      [mdl '/h1']);
    set_param([mdl '/h1'], 'VariableName', 'h1_out', 'SaveFormat', 'Array');
    set_param([mdl '/h2'], 'VariableName', 'h2_out', 'SaveFormat', 'Array');
    % Connect lines: DeltaQ -> SS input
    add_line(mdl, 'DeltaQ/1', 'SS/1',  'autorouting','on');

    % Add Demux to split single vector output into two signals
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux']);
    set_param([mdl '/Demux'], 'Outputs', '2');

    % Reconfigure SS block to output a single 2x1 vector (keep C as is) and
    % connect SS -> Demux -> h1,h2
    add_line(mdl, 'SS/1', 'Demux/1', 'autorouting','on');
    add_line(mdl, 'Demux/1', 'h1/1', 'autorouting','on');
    add_line(mdl, 'Demux/2', 'h2/1', 'autorouting','on');
    set_param(mdl, 'StopTime','3000', 'Solver','ode45');
    save_system(mdl);
end

