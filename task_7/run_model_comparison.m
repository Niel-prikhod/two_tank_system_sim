function save_level_figure(t, h, href, figName, fileName, outFolder)
	f = figure('Name', figName, 'Position', [100 100 900 420]);
	hold on; grid on; box on;
	plot(href(:,1), href(:,2), '--', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'h_{ref}');
	plot(t, h,    '-',  'LineWidth', 1.8, 'DisplayName', 'h');
	xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
	xlabel('Time (s)');
	ylabel('Height (m)');
	title(figName);
	legend('Location', 'northeast');
	ylim([0, 1]);
	if ~exist(outFolder,'dir'), mkdir(outFolder); end
	file = fullfile(outFolder, fileName);
	exportgraphics(f, file, 'Resolution', 300);
	close(f);
end

function save_valve_figure(t, z1, z2, figName, fileName, outFolder)
    f = figure('Name', figName, 'Position', [100 100 900 420]);
    hold on; grid on; box on;

    plot(t, z1, '-', 'LineWidth', 1.8, 'Color', [0 0.4470 0.7410], ...
         'DisplayName', 'z_1');
    plot(t, z2, '-', 'LineWidth', 1.8, 'Color', [0.4660 0.6740 0.1880], ...
         'DisplayName', 'z_2');
    xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    xlabel('Time (s)');
    ylabel('Valve position (-)');
    title(figName);
    legend('Location', 'northeast');
    ylim([0, 1]);

    if ~exist(outFolder, 'dir'), mkdir(outFolder); end
    exportgraphics(f, fullfile(outFolder, fileName), 'Resolution', 300);
    close(f);
end
function save_flow_figure(t, q_in, Q12, Q12ref, Qout, Qoutref, figName, fileName, outFolder)
    

    f = figure('Name', figName, 'Position', [100 100 900 700]);

    % ── Top: inlet and inter-tank flow ───────────────────────────────────
    subplot(2, 1, 1);
    hold on; grid on; box on;

    plot(q_in(:,1), q_in(:,2),    '-',  'LineWidth', 1.5, 'Color', [0.5 0.5 0.5], ...
         'DisplayName', 'Q_{in}');
    plot(t, Q12ref, '--', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], ...
         'DisplayName', 'Q_{12,ref}');
    plot(t, Q12,    '-',  'LineWidth', 1.8, 'Color', [0 0.4470 0.7410], ...
         'DisplayName', 'Q_{12}');
    xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    ylabel('Flow (m^3/s)');
    title(figName);
    legend('Location', 'northeast');

    % ── Bottom: outlet flow ───────────────────────────────────────────────
    subplot(2, 1, 2);
    hold on; grid on; box on;

	plot(q_in(:,1), q_in(:,2),    '-',  'LineWidth', 1.5, 'Color', [0.5 0.5 0.5], ...
         'DisplayName', 'Q_{in}');
    plot(t, Qoutref, '--', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], ...
         'DisplayName', 'Q_{out,ref}');
    plot(t, Qout,    '-',  'LineWidth', 1.8, 'Color', [0.4660 0.6740 0.1880], ...
         'DisplayName', 'Q_{out}');
    xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    xlabel('Time (s)');
    ylabel('Flow (m^3/s)');
    legend('Location', 'northeast');

    if ~exist(outFolder, 'dir'), mkdir(outFolder); end
    exportgraphics(f, fullfile(outFolder, fileName), 'Resolution', 300);
    close(f);
end

close all;
clear;

run('../task_2/params.m'); 
outFolder = fullfile(pwd,'..','docs/task_7');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end
max_time = 10000;
sample_time = 30;
disturbance = 70;
addpath('../task_6');
q_in = signal_gen(Q_ss, max_time, sample_time, disturbance);

controllers = {'pid','mpc'};
results = struct();

for ci = 1:numel(controllers)
	ctrl = controllers{ci};

		mdl = create_model(ctrl); 
        if strcmp(ctrl, 'mpc') && ~exist('mpcobj', 'var')
			if exist('mpc_object.mat', 'file')
				load('mpc_object.mat', 'mpcobj');
			else
				error('mpcobj not found in workspace and ''mpc_object.mat'' does not exist.');
			end
        end
        load('reference.mat');
		load_system(mdl);
		set_param(mdl, 'StopTime', num2str(max_time));
		set_param(mdl, 'ZeroCrossAlgorithm', 'Adaptive');
		set_param(mdl, 'IgnoredZcDiagnostic', 'none');
		set_param(mdl, 'MaxConsecutiveZCs', '10000');
		set_param(mdl, 'ZCThreshold', 'auto');

		simOut = sim(mdl);
		t_sim  = simOut.tout;
		h1_sim = simOut.h1_out;
		h2_sim = simOut.h2_out;
		results.(ctrl).Q12     = simOut.Q12_out;
		results.(ctrl).Qout    = simOut.Qout_out;
		results.(ctrl).Q12ref  = simOut.Q12ref_out;
		results.(ctrl).Qoutref = simOut.Qoutref_out;
		results.(ctrl).z1      = simOut.z1_out;
		results.(ctrl).z2      = simOut.z2_out;
		results.(ctrl).t  = t_sim;
		results.(ctrl).h1 = h1_sim;
		results.(ctrl).h2 = h2_sim;
		results.(ctrl).q_in = q_in;
        try
			close_system(mdl, 0);
		catch
        end
        if ci==1
			figure_in = figure('Name', 'Input Flow');
			grid on;
			plot(q_in(:,1), q_in(:,2));
			ylim([0, 2*Q_ss]);
			xlabel('Time (s)');
			ylabel('Flow (m^3/s)');
			file = fullfile(outFolder, 'input_flow.png');
			exportgraphics(figure_in, file, 'Resolution', 300);
			close(figure_in);
        end
end

for ci = 1:numel(controllers)
	ctrl = controllers{ci};
	
		t_sim = results.(ctrl).t;
		h1 = results.(ctrl).h1;
		h2 = results.(ctrl).h2;

		save_level_figure(t_sim, h1, h1_ref, sprintf('%s - h1', upper(ctrl)), ...
			sprintf('%s_h1.png', ctrl), outFolder);
		save_level_figure(t_sim, h2, h2_ref, sprintf('%s - h2', upper(ctrl)), ...
			sprintf('%s_h2.png', ctrl), outFolder);
		save_valve_figure(t_sim, results.(ctrl).z1, results.(ctrl).z2, ...
			sprintf('%s - valves', upper(ctrl)), sprintf('%s_valves.png', ctrl), outFolder);

		save_flow_figure(t_sim, q_in, results.(ctrl).Q12, ...
			results.(ctrl).Q12ref, results.(ctrl).Qout, results.(ctrl).Qoutref, ...
			sprintf('%s - flows', upper(ctrl)), sprintf('%s_flows.png', ctrl), outFolder);
	end
