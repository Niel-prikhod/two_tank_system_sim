function build_ss_model(k, A)

	% Construct matrix:
	A_mat = [-k/A, 0;
			k/A, -k/A];
	B_mat = [1/A;
				0];
	C_mat = [0, 1];
	D_mat = 0;

    mdl = 'LinearSS';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    % Input: Delta_Q step
    add_block('simulink/Sources/Step',            [mdl '/DeltaQ']);
    set_param([mdl '/DeltaQ'], ...
        'Time',         '200', ...
        'InitialValue', '0',   ...
        'FinalValue',   '0.001');   % +10% placeholder; overridden at runtime

    % State-Space block
    add_block('simulink/Continuous/State-Space',  [mdl '/SS']);
    set_param([mdl '/SS'], ...
        'A', mat2str(A_mat), ...
        'B', mat2str(B_mat), ...
        'C', mat2str(C_mat), ...
        'D', mat2str(D_mat), ...
        'X0', '[0; 0]');           % start at deviation = 0

    % Scope
    add_block('simulink/Sinks/To Workspace',      [mdl '/Out']);
    set_param([mdl '/Out'], 'VariableName', 'ss_out', 'SaveFormat', 'Array');

    add_line(mdl, 'DeltaQ/1', 'SS/1',  'autorouting','on');
    add_line(mdl, 'SS/1',     'Out/1', 'autorouting','on');

    set_param(mdl, 'StopTime','3000', 'Solver','ode45');
    save_system(mdl);
end

