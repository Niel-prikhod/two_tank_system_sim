close all
clear
% Runs nonlinear + SS linear + TF linear for three step sizes.
% Requires: create_linearized_models.m, ../task_1/TwoTankModel.slx, 
% LinearSS.slx, LinearTF.slx

if ~bdIsLoaded('TwoTankModel')
    twoTankPath = fullfile('..','task_1','TwoTankModel.slx');
    if exist(twoTankPath,'file')
        load_system(twoTankPath);
    else
        error('TwoTankModel.slx not found.');
    end
end
open_system('TwoTankModel');

% Open LinearSS if not already open
if ~bdIsLoaded('LinearSS')
    fileFound = which('LinearSS.slx');
    if ~isempty(fileFound)
        load_system(fileFound);
    else
        warning('LinearSS model file not found.');
    end
end
open_system('LinearSS');

% Open LinearTF if not already open
if ~bdIsLoaded('LinearTF')
    fileFound = which('LinearTF.slx');
    if ~isempty(fileFound)
        load_system(fileFound);
    else
        warning('LinearTF model file not found.');
    end
end
open_system('LinearTF');

run('params.m');

% Step sizes to test (as fractions of Q_ss)
step_fractions = [0.10, 0.30, 0.50];
step_labels    = {'+10%', '+30%', '+50%'};
colors_nl  = {'#0072BD', '#D95319', '#77AC30'};   % nonlinear
colors_ss  = {'#4DBEEE', '#EDB120', '#A2142F'};   % state-space
colors_tf  = {'#7E2F8E', '#FF6B6B', '#00A86B'};   % transfer function

T_stop  = 3e4;   % simulate well past settling
T_step  =  1e4;   % step happens at t=200s

figure_1 = figure('Name','Two-Tank Step Comparison','Position',[100 100 1100 750]);

for i = 1:3
    dQ = step_fractions(i) * Q_ss;     % absolute step size in m³/s
    Q_new = Q_ss + dQ;

    %% ── 1. Nonlinear model ───────────────────────────────────────────────
    % Override Q step in TwoTankModel (Step block named 'Qin')
    set_param('TwoTankModel/Qin', ...
        'Time',         num2str(T_step), ...
        'Before', num2str(Q_ss), ...
        'After',   num2str(Q_new));
    set_param('TwoTankModel', 'StopTime', num2str(T_stop));
    set_param('TwoTankModel/h1', 'InitialCondition', num2str(h_ss))
    set_param('TwoTankModel/h2', 'InitialCondition', num2str(h_ss))

    out_nl = sim('TwoTankModel');
    t_nl   = out_nl.tout;
    h2_nl  = out_nl.h2;            % absolute h2 from nonlinear model

    %% ── 2. State-space linear model ──────────────────────────────────────
    set_param('LinearSS/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'Before', '0', ...
        'After', num2str(dQ));    
    set_param('LinearSS', 'StopTime', num2str(T_stop));

    out_ss  = sim('LinearSS');
    t_ss    = out_ss.tout;
    h2_ss_d = out_ss.ss_out;          
    h2_ss_abs = h2_ss_d + h_ss;        % convert back to absolute

    %% ── 3. Transfer-function linear model ───────────────────────────────
    set_param('LinearTF/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'Before', '0', ...
        'After', num2str(dQ));
    set_param('LinearTF', 'StopTime', num2str(T_stop));

    out_tf  = sim('LinearTF');
    t_tf    = out_tf.tout;
    h2_tf_d = out_tf.tf_out;
    h2_tf_abs = h2_tf_d + h_ss;

    %% ── Plot h2 response ─────────────────────────────────────────────────
    subplot(1,3,i);
    hold on; grid on; box on;

    plot(t_nl/1, h2_nl.Data,     '-',  'Color', colors_nl{i}, 'LineWidth', 2.0, ...
         'DisplayName', 'Nonlinear');
    plot(t_ss/1, h2_ss_abs, '--', 'Color', colors_ss{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear SS');
    plot(t_tf/1, h2_tf_abs, ':',  'Color', colors_tf{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear TF');

    xlabel('Time (s)');
    ylabel('h_2 (m)');
    title(sprintf('Step %s  (\\DeltaQ = %.4f m^3/s)', step_labels{i}, dQ));
    legend('Location','southeast','FontSize',8);

    xlim([0, T_stop]);              % x in seconds

    % Set y-limits per subplot: first 0-1, second 0-1.5, third 0-2

    ylim([0, 2]);

    yticks(0:0.5:2);                % ticks every 0.1
    xticks(linspace(0, T_stop, 6));
end

sgtitle('Two-Tank h_2 Response: Nonlinear vs Linear SS vs Linear TF', ...
        'FontSize',14,'FontWeight','bold');

outFolder = fullfile(pwd,'..','docs');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end

file = fullfile(outFolder, 'task_2_comp.png');
exportgraphics(figure_1, file, 'Resolution', 300);


%% ── Summary table in command window ─────────────────────────────────────
% fprintf('\n%-10s %-18s %-18s\n', 'Step', 'New Q (m^3/s)', 'Predicted h2_ss (m)');
% fprintf('%s\n', repmat('-',1,48));
% for i = 1:3
%     dQ      = step_fractions(i) * Q_ss;
%     Q_new   = Q_ss + dQ;
%     h2_pred = h_ss + dcgain(tf(k, [A^2,2*A*k,k^2])) * dQ;
%     fprintf('%-10s %-18.5f %-18.4f\n', step_labels{i}, Q_new, h2_pred);
% end

close_system('LinearSS', 0);
close_system('LinearTF', 0);
close_system('TwoTankModel', 0);