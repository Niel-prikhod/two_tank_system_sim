function [dh1, dh2] = two_tank_ode(h1, h2, Q_in, z1, z2)
%   Inputs:
%     h1, h2  — current water heights [m]
%     Q_in    — inlet flow to tank 1 [m3/s]
%     z1      — actual position of valve 1 (tank 1 outlet), [0,1]
%     z2      — actual position of valve 2 (tank 2 outlet), [0,1]
%
%   Outputs:
%     dh1, dh2 — rate of change of heights [m/s]

    A  = 4;       % m²   tank cross-section
    Kv = 0.005;   % m²   valve flow factor
    g  = 9.81;    % m/s²

    % Flow through valve 1 — depends only on h1 (tank 2 sits below)
    Q12  = Kv * z1 * sqrt(2 * g * max(h1, 0));

    % Flow through valve 2 — depends only on h2
    Qout = Kv * z2 * sqrt(2 * g * max(h2, 0));

    dh1 = (Q_in - Q12)  / A;
    dh2 = (Q12  - Qout) / A;
end
