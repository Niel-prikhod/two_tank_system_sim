function save_level_figure(t, h, href, figName, fileName, outFolder)
	f = figure('Name', figName, 'Position', [100 100 900 420]);
	hold on; grid on; box on;
	plot(t, href, '--', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'h_{ref}');
	plot(t, h,    '-',  'LineWidth', 1.8, 'DisplayName', 'h');
	xline(500, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
	xlabel('Time (s)');
	ylabel('Height (m)');
	title(figName);
	legend('Location', 'southeast');
	ylim([0.8, 1]);
	if ~exist(outFolder,'dir'), mkdir(outFolder); end
	file = fullfile(outFolder, fileName);
	exportgraphics(f, file, 'Resolution', 300);
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
		if exist('t','var') && numel(t)==numel(href) && ~isempty(href)
			href_interp = interp1(t, href, t_sim, 'linear', 'extrap');
		else
			href_interp = h_ss * ones(size(t_sim));
		end
		save_level_figure(t_sim, h1, href_interp, sprintf('%s - h1', upper(ctrl)), ...
			sprintf('%s_h1.png', ctrl), outFolder);
		save_level_figure(t_sim, h2, href_interp, sprintf('%s - h2', upper(ctrl)), ...
			sprintf('%s_h2.png', ctrl), outFolder);
	end
