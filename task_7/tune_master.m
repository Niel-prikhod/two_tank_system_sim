function [Kp_m, Ki_m] = tune_master(A, CL_slave)
	G_master  = tf(1, [A, 0]);
	G_m_total = G_master * CL_slave;
	C_master = pidtune(G_m_total, 'PI');
	Kp_m = C_master.Kp;   Ki_m = C_master.Ki;
end
