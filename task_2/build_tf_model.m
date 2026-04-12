function build_tf_model(k, A)
	% construct transfer function
	num_tf = k;
	den_tf = [A^2,  2*A*k,  k^2];

    mdl = 'LinearTF';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    % Input: Delta_Q step
    add_block('simulink/Sources/Step',              [mdl '/DeltaQ']);
    set_param([mdl '/DeltaQ'], ...
        'Time',         '200', ...
        'InitialValue', '0',   ...
        'FinalValue',   '0.001');

    % Transfer Function block
    add_block('simulink/Continuous/Transfer Fcn',   [mdl '/TF']);
    set_param([mdl '/TF'], ...
        'Numerator',   mat2str(num_tf), ...
        'Denominator', mat2str(den_tf));

    % Output logger
    add_block('simulink/Sinks/To Workspace',        [mdl '/Out']);
    set_param([mdl '/Out'], 'VariableName', 'tf_out', 'SaveFormat', 'Array');

    add_line(mdl, 'DeltaQ/1', 'TF/1',  'autorouting','on');
    add_line(mdl, 'TF/1',     'Out/1', 'autorouting','on');

    set_param(mdl, 'StopTime','3000', 'Solver','ode45');
    save_system(mdl);
end
