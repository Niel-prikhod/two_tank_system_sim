% simulation
model = 'TwoTankModel';
if bdIsLoaded(model)
	close_system(model, 0);
end
open_system(model);
simOut = sim(model, 'StopTime', '6000', 'SaveTime', 'on');

% extract data from simulation
t = simOut.tout;
h1_data = simOut.h1;
h2_data = simOut.h2;
q12_data = simOut.Q12;
q2_data = simOut.Q;
close_system(model, 0);

outFolder = fullfile(pwd,'..','docs');
if ~exist(outFolder,'dir')    
	mkdir(outFolder);
end

%plot data
figure_1 = figure('Visible', 'off');
plot(t, h2_data.Data, 'b-', 'LineWidth', 1.5); hold on;
plot(t, h1_data.Data, 'g-', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Water level (m)');
title('Water level in both tanks');
yline(0.816, 'r--', 'Steady state');
legend({'h_1','h_2'},'Location','best');
grid on;

file1 = fullfile(outFolder,'tank_levels.png');
exportgraphics(figure_1, file1, 'Resolution', 300);
close(figure_1);
ylim([0 0.85]);

figure_2 = figure('Visible', 'off');
plot(t, q12_data.Data, 'm-', 'LineWidth', 1.5); hold on;
plot(t, q2_data.Data, 'c-', 'LineWidth', 1.5); 
xlabel('Time (s)');
ylabel('Flow rate (m^3/s)');
title('Water level in both tanks');
yline(10^-2, 'r--', 'Steady state');
legend({'Q','Q_{12}'},'Location','best');
grid on;
ylim([0 0.012]);
ax = gca;                                 
handleax.YAxis.Exponent = 0;                    
labelax.YAxis.TickLabelFormat = '%0.2e';

file2 = fullfile(outFolder, 'flows.png');
exportgraphics(figure_2, file2, 'Resolution', 300);
close(figure_2);
