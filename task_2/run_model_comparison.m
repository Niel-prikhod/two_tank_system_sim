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

% Prepare figures for h1 and Q_out
figure_h1 = figure('Name','Two-Tank h_1 Comparison','Position',[120 120 900 600]);
figure_Q  = figure('Name','Two-Tank Q_{out} Comparison','Position',[140 140 900 600]);

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
    h1_nl  = out_nl.h1;            % absolute h1 from nonlinear model
    Qout_nl = out_nl.Q;        % Q_out (assumes logged as Qout)

    %% ── 2. State-space linear model ──────────────────────────────────────
    set_param('LinearSS/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'Before', '0', ...
        'After', num2str(dQ));    
    set_param('LinearSS', 'StopTime', num2str(T_stop));

    out_ss  = sim('LinearSS');
    t_ss    = out_ss.tout;
    h2_ss_d = out_ss.h2_out;          
    h2_ss_abs = h2_ss_d + h_ss;        % convert back to absolute
    h1_ss_d = out_ss.h1_out;            % assumes linear SS logs delta h1 as ss_h1
    h1_ss_abs = h1_ss_d + h_ss;
    
    Qout_ss = Q_ss + k * h2_ss_d;
    
    %% ── 3. Transfer-function linear model ───────────────────────────────
    set_param('LinearTF/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'Before', '0', ...
        'After', num2str(dQ));
    set_param('LinearTF', 'StopTime', num2str(T_stop));

    out_tf  = sim('LinearTF');
    t_tf    = out_tf.tout;
    h2_tf_d = out_tf.h2_out;
    h2_tf_abs = h2_tf_d + h_ss;
    h1_tf_d = out_tf.h1_out;            % assumes TF logs delta h1 as tf_h1
    h1_tf_abs = h1_tf_d + h_ss;
    
    Qout_tf = Q_ss + k * h2_tf_d;


    %% ── Plot h2 response ─────────────────────────────────────────────────
    figure(figure_1);
    subplot(1,3,i);
    hold on; grid on; box on;

    plot(t_nl, h2_nl.Data,     '-',  'Color', colors_nl{i}, 'LineWidth', 2.0, ...
         'DisplayName', 'Nonlinear');
    plot(t_ss, h2_ss_abs, '--', 'Color', colors_ss{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear SS');
    plot(t_tf, h2_tf_abs, ':',  'Color', colors_tf{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear TF');

    xlabel('Time (s)');
    ylabel('h_2 (m)');
    title(sprintf('Step %s  (\\DeltaQ = %.4f m^3/s)', step_labels{i}, dQ));
    legend('Location','southeast','FontSize',8);

    xlim([0, T_stop]);              % x in seconds
    ylim([0, 2]);
    yticks(0:0.5:2);
    xticks(linspace(0, T_stop, 6));

    %% ── Plot h1 response on its own figure ───────────────────────────────
    figure(figure_h1);
    subplot(1,3,i);
    hold on; grid on; box on;

    % Nonlinear h1: may be timeseries or numeric
    if isstruct(h1_nl) || isa(h1_nl,'timeseries')
        h1_nl_data = h1_nl.Data;
    else
        h1_nl_data = h1_nl;
    end

    plot(t_nl, h1_nl_data,     '-',  'Color', colors_nl{i}, 'LineWidth', 2.0, ...
         'DisplayName', 'Nonlinear');
    plot(t_ss, h1_ss_abs, '--', 'Color', colors_ss{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear SS');
    plot(t_tf, h1_tf_abs, ':',  'Color', colors_tf{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear TF');

    xlabel('Time (s)');
    ylabel('h_1 (m)');
    title(sprintf('Step %s  (\\DeltaQ = %.4f m^3/s)', step_labels{i}, dQ));
    legend('Location','southeast','FontSize',8);

    xlim([0, T_stop]);
    ylim([0, 2]);
    yticks(0:0.5:2);
    xticks(linspace(0, T_stop, 6));

    %% ── Plot Q_out response on its own figure ────────────────────────────
    figure(figure_Q);
    subplot(1,3,i);
    hold on; grid on; box on;

    % Nonlinear Qout: may be timeseries or numeric
    if isstruct(Qout_nl) || isa(Qout_nl,'timeseries')
        Qout_nl_data = Qout_nl.Data;
    else
        Qout_nl_data = Qout_nl;
    end

    plot(t_nl, Qout_nl_data,     '-',  'Color', colors_nl{i}, 'LineWidth', 2.0, ...
         'DisplayName', 'Nonlinear');
    plot(t_ss, Qout_ss_abs, '--', 'Color', colors_ss{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear SS');
    plot(t_tf, Qout_tf_abs, ':',  'Color', colors_tf{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear TF');

    xlabel('Time (s)');
    ylabel('Q_{out} (m^3/s)');
    title(sprintf('Step %s  (\\DeltaQ = %.4f m^3/s)', step_labels{i}, dQ));
    legend('Location','southeast','FontSize',8);

    xlim([0, T_stop]);
    % autoscale Q axis a bit around steady-state
    qmin = min([min(Qout_nl_data), min(Qout_ss_abs), min(Qout_tf_abs)]);
    qmax = max([max(Qout_nl_data), max(Qout_ss_abs), max(Qout_tf_abs)]);
    margin = 0.1*(qmax-qmin + eps);
    ylim([qmin-margin, qmax+margin]);
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