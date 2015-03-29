%% Create the system
nu = 2;

A = [-1 0 ;
     0 -1];
B = eye(nu);
C = eye(nu);
D = [];

SG= STLC_lti(A,B,C,D); 
SG.x0= [0 ; 0];

%% Controller Initialisation
% Time
SG.time = 0:1:100; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=7;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;
Sys.lambda_rho = 10;

%% STL formula

%SG.stl_list = {'ev_[0, 5] (u1(t) > 0.1)'};
%SG.stl_list = {'ev_[3,5] (y1(t) > 5 and y2(t)<-2)'};
SG.stl_list{1} = '( (x1(t)<1  =>  ev_[0, 8] (x1(t)>3)) and ((x1(t)>3)  =>  ev_[0, 8] (x1(t)<1)) )';
%SG.stl_list{1} = '( y1(t)<1  =>  ev_[0, 10] (y1(t)>2)) and ((y1(t)>2)  =>  ev_[0, 10] (y1(t)<1))';
%SG.stl_list{1} = 'alw_[0,Inf] ( y1(t)<11 and ev_[0,5](y1(t)>0) and ev_[0,5](y1(t)<1))';


SG.min_rob = 0.01;    
SG.bigM = 1000;

%% Initial state

%return;
%% running stuff
fprintf('Computing controller...');
tic;
controller = get_controller(SG,'interval');toc;
%run_open_loop(SG, controller);
run_deterministic(SG, controller);
