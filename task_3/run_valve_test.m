close all;
clear;
clc;

%% run_simulation.m
% Tests TwoTankValveModel by stepping valves at defined times.
% Plots h1 and h2.

run('../task_2/params.m');

mdl   = 'model_w_valves';
if bdIsLoaded(mdl) == 0, load_system(mdl); end

%% Simulation settings
T_stop  = 3000;   % s — total simulation time
T_step  =  500;   % s — when valves change

%% Valve commands
% z_ss = 0.5 is the nominal operating point
% Change these to test different scenarios
z1_initial = 0.5;   % valve 1 before step
z1_final   = 0.8;   % valve 1 after step  — open further

z2_initial = 0.5;   % valve 2 before step
z2_final   = 0.3;   % valve 2 after step  — partially close

%% Configure step blocks
set_param([mdl '/z1_cmd'], ...
    'Time',         num2str(T_step), ...
    'Before', num2str(z1_initial), ...
    'After',   num2str(z1_final));

set_param([mdl '/z2_cmd'], ...
    'Time',         num2str(T_step), ...
    'Before', num2str(z2_initial), ...
    'After',   num2str(z2_final));

set_param(mdl, 'StopTime', num2str(T_stop));

%% Run
simOut = sim(mdl);

t  = simOut.tout;
h1 = simOut.h1_out;
h2 = simOut.h2_out;
z1 = simOut.z1_out;
z2 = simOut.z2_out;

%% Plot
figure_1 = figure('Name', 'Two-Tank Valve Test', 'Position', [100 100 900 600]);

% h1 and h2
subplot(2, 1, 1);
hold on; grid on; box on;

plot(t, h1, 'b-',  'LineWidth', 1.8, 'DisplayName', 'h_1');
plot(t, h2, 'r-',  'LineWidth', 1.8, 'DisplayName', 'h_2');
yline(h_ss, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xline(T_step, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

ylabel('Height (m)');
title('Tank water levels');
legend('Location', 'best');
ylim([0, max(max(h1), max(h2)) * 1.15]);

% Valve positions
subplot(2, 1, 2);
hold on; grid on; box on;

plot(t, z1, 'b-',  'LineWidth', 1.8, 'DisplayName', 'z_1  (valve 1)');
plot(t, z2, 'r-',  'LineWidth', 1.8, 'DisplayName', 'z_2  (valve 2)');
xline(T_step, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

ylabel('Valve position (−)');
xlabel('Time (s)');
title('Valve positions (actual, after 1st-order lag)');
legend('Location', 'best');
ylim([0, 1.05]);

outFolder = fullfile(pwd,'..','docs/task_3');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end

file = fullfile(outFolder, 'valve_function.png');
exportgraphics(figure_1, file, 'Resolution', 300);

close_system(mdl, 0);
