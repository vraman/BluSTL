clear;
close all;

%% Create the system
nu = 2;
nw = 2;
SG = signal_generator(nu,nw);

%% Controller Initialisation
% Time
SG.time = 0:1:100; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=7;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;
SG.lambda_rho = 0.1;

SG.Wref = [ cos(SG.time) ; 0.0*cos(SG.time)+1.1;];

phis= { '(w1(t)>0) or (w2(t)>0)', ...    % Heisenbug #1: this does not work 
    'ev_[0,1.1] (w1(t)>0)', ...
    '((w1(t)>0)) or (ev_[0,1.1] (w1(t)>0))',...    
    'alw_[0,5] (w1(t)>0 )',...
    'not (alw_[0,5] (w1(t)>0))',...
    'alw_[0,1.1] alw_[0,1.1] (w1(t)>0)'}; % Heisenbug... (infeasible for L=7,10, works for L=1..6, 8,9

for i = 6
    phi = phis{i};
    rob= monitor(SG, phi)
    rob_int = monitor_interval(SG, phi)
end
