function	ref = set_reference()
	sample_time = 30;      
	total_time = 10000;         
	initial_value = 0;         
	peak_value = 1.0;          
	rise_time = 5000;              
	hold_time = 0;              
	fall_time = 5000;                

	num_cycles = 1;                 
	reset_between_cycles = false;  
	t = 0:sample_time:total_time;
	t = t'; 
	num_samples = length(t);
	reference_signal = zeros(num_samples, 1);
	cycle_duration = rise_time + hold_time + fall_time;
	for sample_idx = 1:num_samples
		current_time = t(sample_idx);
		cycle_number = floor(current_time / cycle_duration) + 1;
		time_in_cycle = mod(current_time, cycle_duration);
		if cycle_number > num_cycles
			reference_signal(sample_idx) = initial_value;
		else
			if time_in_cycle < rise_time
				slope = (peak_value - initial_value) / rise_time;
				reference_signal(sample_idx) = initial_value + slope * time_in_cycle;
			elseif time_in_cycle < (rise_time + hold_time)
				reference_signal(sample_idx) = peak_value;
			else
				time_in_fall = time_in_cycle - rise_time - hold_time;
				slope = (initial_value - peak_value) / fall_time;
				reference_signal(sample_idx) = peak_value + slope * time_in_fall;
			end
		end
	end

	ref = [t, reference_signal];
end
