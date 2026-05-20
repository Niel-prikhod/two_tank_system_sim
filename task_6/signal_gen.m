function out = signal_gen(offset, max_time, sample_time, disturbance)
	rand_seed = 42;
    rng(rand_seed);
	change_interval = 600;
	t_ctrl = (0:change_interval:max_time)';
	t = (0:sample_time:max_time)';
	n_ctrl = length(t_ctrl);
	control_points = ((2*rand(n_ctrl, 1) - 1) * (disturbance / 100) + 1) * offset;
	disturbed_signal = interp1(t_ctrl, control_points, t, 'pchip');
	min_limit = 0;
	max_limit = 2 * offset;
	disturbed_signal = max(disturbed_signal, min_limit);   
	disturbed_signal = min(disturbed_signal, max_limit);  
	out = [t, disturbed_signal];
end
