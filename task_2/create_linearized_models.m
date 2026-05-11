% common parameters:
g = 9.81; % [m/s2]
tank_area = 4; % [m2]
cross_section = 0.0025; % [m2]
height_ss = 0.816; % [m]
flow_ss = 0.01; % [m3/s]

lin_koef = cross_section * sqrt(2 * g) / 2 / sqrt(height_ss);

build_ss_model(lin_koef, tank_area);
build_tf_model(lin_koef, tank_area);
