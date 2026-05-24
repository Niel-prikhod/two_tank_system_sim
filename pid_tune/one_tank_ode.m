function	[level_change, q_out] = one_tank_ode(level, action, q_in)
	Kv = 0.005;
	g = 9.81;
	A = 4.0;
	q_out = Kv * action * sqrt(2 * g * max(level, 0));
	level_change = (q_in - q_out) / A; 
end
