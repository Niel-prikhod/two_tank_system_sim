%% run_cascade.m
% Tests cascade PI control with a reference step.
% Plots h1, h2, and h_ref on a single figure.

run('../task_2/params.m');

mdl    = 'model_w_cascade';
T_stop = 5000;   % s — long enough to see full settling
T_step =  500;   % s — reference step time

load_system(mdl);

%% Configure reference step
h_ref_new = 1.2 * h_ss;   % +20% level setpoint

set_param([mdl '/h_ref'], ...
    'Time',         num2str(T_step),  ...
    'Before', num2str(h_ss),    ...
    'After',   num2str(h_ref_new));

set_param(mdl, 'StopTime', num2str(T_stop));


%% Run simulation
set_param(mdl, 'ZeroCrossAlgorithm', 'Adaptive');
simOut = sim(mdl);

t    = simOut.tout;
h1   = simOut.h1_out;
h2   = simOut.h2_out;
href = simOut.href_out;

close_system(mdl, 0);

%% Plot — single figure
figure_1 = figure('Name', 'Cascade PI — Tank Levels', 'Position', [100 100 900 420]);
hold on; grid on; box on;

plot(t, href, 'k--', 'LineWidth', 1.5, 'DisplayName', 'h_{ref}', 'Color', [0.8500 0.3250 0.0980]);
plot(t, h1,   'b-',  'LineWidth', 1.8, 'DisplayName', 'h_1  (Tank 1)');
plot(t, h2,   'r-',  'LineWidth', 1.8, 'DisplayName', 'h_2  (Tank 2)');

xline(T_step, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('Water level (m)');
title('Cascade PI — response to reference step');
legend('Location', 'southeast');

ylim([h_ss * 0.9, h_ref_new * 1.1]);


outFolder = fullfile(pwd,'..','docs/task_4');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end

file = fullfile(outFolder, 'cascade_pid.png');
exportgraphics(figure_1, file, 'Resolution', 300);
