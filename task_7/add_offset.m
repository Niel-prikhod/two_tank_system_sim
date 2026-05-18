function new = add_offset(reference_signal, time_offset)
	sample_time = 10;      
	total_time = 10000;         
	initial_value = 0;         
	peak_value = 1.0;          
	rise_time = 5000;              
	hold_time = 0;              
	fall_time = 5000;                
	new = reference_signal;

	for i = 1:length(new(:,1))
		if  new(i,1) < time_offset
			new(i, 2) = 0;
		else
			time_in_cycle = new(i,1) - time_offset;
			if time_in_cycle < rise_time
				slope = (peak_value - initial_value) / rise_time;
				new(i, 2) = initial_value + slope * time_in_cycle;
			elseif time_in_cycle < (rise_time + hold_time)
				new(i, 2) = peak_value;
			elseif time_in_cycle < (rise_time + hold_time + fall_time)
				time_in_fall = time_in_cycle - rise_time - hold_time;
				slope = (initial_value - peak_value) / fall_time;
				new(i, 2) = peak_value + slope * time_in_fall;
			else
				new(i, 2) = initial_value;
			end
		end
	end
end
