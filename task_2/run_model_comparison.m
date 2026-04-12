% Runs nonlinear + SS linear + TF linear for three step sizes.
% Requires: create_linearized_models.m, ../task_1/TwoTankModel.slx, 
% .slx, LinearTF.slx

run('params.m');

% Step sizes to test (as fractions of Q_ss)
step_fractions = [0.10, 0.30, 0.50];
step_labels    = {'+10%', '+30%', '+50%'};
colors_nl  = {'#0072BD', '#D95319', '#77AC30'};   % nonlinear
colors_ss  = {'#4DBEEE', '#EDB120', '#A2142F'};   % state-space
colors_tf  = {'#7E2F8E', '#FF6B6B', '#00A86B'};   % transfer function

T_stop  = 5000;   % simulate well past settling
T_step  =  200;   % step happens at t=200s

figure('Name','Two-Tank Step Comparison','Position',[100 100 1100 750]);

for i = 1:3
    dQ = step_fractions(i) * Q_ss;     % absolute step size in m³/s
    Q_new = Q_ss + dQ;

    %% ── 1. Nonlinear model ───────────────────────────────────────────────
    % Override Q step in TwoTankModel (Step block named 'Qin')
    set_param('TwoTankModel/Qin', ...
        'Time',         num2str(T_step), ...
        'InitialValue', num2str(Q_ss), ...
        'FinalValue',   num2str(Q_new));
    set_param('TwoTankModel', 'StopTime', num2str(T_stop));

    out_nl = sim('TwoTankModel');
    t_nl   = out_nl.tout;
    h2_nl  = out_nl.h2_out;            % absolute h2 from nonlinear model

    %% ── 2. State-space linear model ──────────────────────────────────────
    set_param('LinearSS/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'FinalValue', num2str(dQ));     % deviation input
    set_param('LinearSS', 'StopTime', num2str(T_stop));

    out_ss  = sim('LinearSS');
    t_ss    = out_ss.tout;
    h2_ss_d = out_ss.ss_out;           % deviation Delta_h2
    h2_ss_abs = h2_ss_d + h_ss;        % convert back to absolute

    %% ── 3. Transfer-function linear model ───────────────────────────────
    set_param('LinearTF/DeltaQ', ...
        'Time',       num2str(T_step), ...
        'FinalValue', num2str(dQ));
    set_param('LinearTF', 'StopTime', num2str(T_stop));

    out_tf  = sim('LinearTF');
    t_tf    = out_tf.tout;
    h2_tf_d = out_tf.tf_out;
    h2_tf_abs = h2_tf_d + h_ss;

    %% ── Plot h2 response ─────────────────────────────────────────────────
    subplot(1,3,i);
    hold on; grid on; box on;

    plot(t_nl/60, h2_nl,     '-',  'Color', colors_nl{i}, 'LineWidth', 2.0, ...
         'DisplayName', 'Nonlinear');
    plot(t_ss/60, h2_ss_abs, '--', 'Color', colors_ss{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear SS');
    plot(t_tf/60, h2_tf_abs, ':',  'Color', colors_tf{i}, 'LineWidth', 1.8, ...
         'DisplayName', 'Linear TF');

    % Mark new steady state (linear prediction)
    h2_new_ss = h_ss + (dQ / (2*k));   % from DC gain of G2: k/(2k)^2·... 
    % Actually DC gain of G2 = k/k^2 = 1/k, same as G1
    h2_new_lin = h_ss + dcgain(tf(k, [A^2, 2*A*k, k^2])) * dQ;
    yline(h2_new_lin, 'k-.', 'LineWidth', 1.0);

    xline(T_step/60, 'k:', 'LineWidth', 1.0);

    xlabel('Time (min)');
    ylabel('h_2 (m)');
    title(sprintf('Step %s  (\\DeltaQ = %.4f m^3/s)', step_labels{i}, dQ));
    legend('Location','southeast','FontSize',8);
    ylim([h_ss*0.95, h2_new_lin*1.10]);
end

sgtitle('Two-Tank h_2 Response: Nonlinear vs Linear SS vs Linear TF', ...
        'FontSize',14,'FontWeight','bold');

%% ── Summary table in command window ─────────────────────────────────────
fprintf('\n%-10s %-18s %-18s\n', 'Step', 'New Q (m^3/s)', 'Predicted h2_ss (m)');
fprintf('%s\n', repmat('-',1,48));
for i = 1:3
    dQ      = step_fractions(i) * Q_ss;
    Q_new   = Q_ss + dQ;
    h2_pred = h_ss + dcgain(tf(k, [A^2,2*A*k,k^2])) * dQ;
    fprintf('%-10s %-18.5f %-18.4f\n', step_labels{i}, Q_new, h2_pred);
end
