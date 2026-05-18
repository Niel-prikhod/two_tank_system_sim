close all;
run('../task_2/params.m'); 
mdl = 'mpc_model';
load_system(mdl);
set_param([mdl '/h_ref'], ...
    'Time',   '500',               ...
    'Before', num2str(h_ss),       ...
    'After',  num2str(1.2 * h_ss));

max_time = 5000;
sample_time = 30;
set_param(mdl, 'StopTime', num2str(max_time));

disturbance = 70;
q_in = signal_gen(Q_ss, max_time, sample_time, disturbance);


model = 'mpc_model';

set_param(model, 'ZeroCrossAlgorithm', 'Adaptive');
set_param(model, 'IgnoredZcDiagnostic', 'none');
set_param(model, 'MaxConsecutiveZCs', '10000');
set_param(model, 'ZCThreshold', 'auto');

simOut = sim(mdl);

t    = simOut.tout;
h1   = simOut.h1_out;
h2   = simOut.h2_out;
href = simOut.href_out;

figure_in = figure('Name', 'Input Flow');
grid on;
plot(q_in(:,1), q_in(:,2));
ylim([0, 2*Q_ss]);
xlabel('Time (s)');
ylabel('Flow (m^3/s)');

outFolder = fullfile(pwd,'..','docs/task_6');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end

file = fullfile(outFolder, 'disturbance_input.png');
exportgraphics(figure_in, file, 'Resolution', 300);


figure_1 = figure('Name', 'MPC + Cascade Slave PI', 'Position', [100 100 900 420]);
hold on; grid on; box on;

plot(t, href, 'k--', 'LineWidth', 1.5, 'DisplayName', 'h_{ref}', 'Color', [0.8500 0.3250 0.0980]);
plot(t, h1,   'b-',  'LineWidth', 1.8, 'DisplayName', 'h_1');
plot(t, h2,   'r-',  'LineWidth', 1.8, 'DisplayName', 'h_2');

xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

xlabel('Time (s)');
ylabel('Height (m)');
title('MPC master + cascade slave PI — level response');
legend('Location', 'southeast');
ylim([h_ss * 0.95, 1.2 * h_ss * 1.05]);

file = fullfile(outFolder, 'regulate_disturbance.png');
exportgraphics(figure_1, file, 'Resolution', 300);
