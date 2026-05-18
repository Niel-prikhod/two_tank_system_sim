function out = signal_gen(offset, max_time, sample_time, disturbance)
	rand_seed = 42;
    rng(rand_seed);
	t = (0:sample_time:max_time)';
	base_signal = ones(length(t), 1) * offset;
	min_limit = 0;
	max_limit = 2 * offset;
	disturbance_factor = 1 + (2*rand(length(t), 1) - 1) * (disturbance / 100);
	disturbed_signal = base_signal .* disturbance_factor;
	disturbed_signal = max(disturbed_signal, min_limit);   
	disturbed_signal = min(disturbed_signal, max_limit);  
	out = [t, disturbed_signal];
end
