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
        'Before', '0',   ...
        'After',   '0.001');

    % Transfer Function blocks
    add_block('simulink/Continuous/Transfer Fcn',   [mdl '/h2']);
    set_param([mdl '/h2'], ...
        'Numerator',   mat2str(num_tf), ...
        'Denominator', mat2str(den_tf));

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/h1']);
    set_param([mdl '/h1'], ...
        'Numerator',   '1', ...
        'Denominator', mat2str([A, k])); 

    % Output loggers for both TF blocks
    add_block('simulink/Sinks/To Workspace',        [mdl '/h1_out']);
    set_param([mdl '/h1_out'], 'VariableName', 'h1_out', 'SaveFormat', 'Array');

    add_block('simulink/Sinks/To Workspace',        [mdl '/h2_out']);
    set_param([mdl '/h2_out'], 'VariableName', 'h2_out', 'SaveFormat', 'Array');

    % Connect routing: DeltaQ -> h1 and h2 in parallel, then to respective outputs
    add_line(mdl, 'DeltaQ/1', 'h1/1',  'autorouting','on');
    add_line(mdl, 'DeltaQ/1', 'h2/1',  'autorouting','on');

    add_line(mdl, 'h1/1',     'h1_out/1', 'autorouting','on');
    add_line(mdl, 'h2/1',     'h2_out/1', 'autorouting','on');

    set_param(mdl, 'StopTime','3000', 'Solver','ode45');
    save_system(mdl);
end
