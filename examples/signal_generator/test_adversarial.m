%% Create the system
nu = 2;
nw = 2;
SG = signal_generator(nu,nw);

%% Controller Initialisation
% Time
SG.time = 0:.1:100; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=7;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;
SG.lambda_rho = 0.1;

SG.Wref = [sin(SG.time); cos(SG.time)];
SG.w_ub(:) = .1;
SG.w_lb(:) = -.1;

%% STL formula

%% TODO: make it so the following would work:
% SG.stl_list = {'ev_[3,5] ([y1(t);-y2(t)] > [5;2])'};
% SG.stl_list = {'ev_[3,5] (Y(1:2,t) > 2)'};

%SG.stl_list{1} = 'ev_[0, 6] (y1(t)>3)';
SG.stl_list{1} = 'alw ( y1(t)<1 => ev_[0, 10] (y1(t)>3)) and ((y1(t)>3)  =>  ev_[0, 10] (y1(t)<1))';
%SG.stl_list{1} = 'alw (y1(t)<1 => ev_[0, 10] (y1(t)>3) )';
% SG.stl_list{1} = 'alw_[0,Inf] ( y1(t)<11 and ev_[0,5](y1(t)>0) and ev_[0,5](y1(t)<1))';

SG.min_rob = 0.01;    
SG.bigM = 1000;

SG.plot_u = [1];
SG.plot_y =[1];
SG.plot_w = [1];

%% Initial state

%return;
%% running stuff
fprintf('Computing controller...');
tic;
SG.controller = get_controller(SG,'robust');
SG.adversary = get_adversary(SG);
toc;
SG = SG.reset_data();

[SG, status] = compute_input(SG, SG.controller);
[SG, status] = compute_disturbance(SG, SG.adversary);
