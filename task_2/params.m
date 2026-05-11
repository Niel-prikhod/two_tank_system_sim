%% Physical parameters
A   = 4;        % m²   tank cross-section
a   = 0.0025;   % m²   orifice cross-section
g   = 9.81;     % m/s²

%% Steady-state operating point
h_ss  = 0.816;          % m
Q_ss  = 0.01;           % m³/s

%% Linearisation gain  k = a*sqrt(2g) / (2*sqrt(h_ss))
k     = a * sqrt(2*g) / (2 * sqrt(h_ss));   % ~0.006128 m²/s
tau   = A / k;                               % ~652.6 s  (time constant)
