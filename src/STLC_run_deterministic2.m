function [Sys, params, rob] = STLC_run_deterministic(Sys, controller)
% STLC_run_deterministic    runs a receding horizon control problem 
%                           for the system described by Sys, using the
%                           provided controller optimizer object, using the 
%                           horizon defined by Sys.L
%                           
% Input: 
%       Sys: an STLC_lti instance
%       controller: a YALMIP opmitizer object representing the system's 
%                   optimization problem
%
% Output: 
%       Sys: modified with additional system_data
%       params: controller data
%
% :copyright: TBD
% :license: TBD

global StopRequest
StopRequest=0;

rob = Sys.min_rob;

%% Time
sys = Sys.sys;
ts=Sys.ts; % sampling time for the model
sys_ts = Sys.time(2)-Sys.time(1); % sampling time for the system
L=Sys.L;  % horizon (# of steps)
Sys.time_d = (0:2*L-1)*ts; % discretized time for the controller


%% System dimensions and variables
Sys.sysd = c2d(sys,ts);

x0 = Sys.x0;
nu=Sys.nu;
nx=Sys.nx;
nw=Sys.nw;
ny=Sys.ny;

% reference disturbance
if isempty(Sys.Wref)
    Sys.Wref = 0*time;
end
Wn = sensing(Sys);


%% Make data nb_stages times longer
nb_stages=Sys.nb_stages;
time = Sys.time;
ntime = zeros(1, nb_stages*numel(time));
for istage = 0:nb_stages-1
    ntime(istage*numel(time)+1:(istage+1)*numel(time))= time+istage*(time(end)+time(2)) ;
end
Sys.time = ntime;
time = Sys.time;
Sys.Wref = repmat(Sys.Wref,1,Sys.nb_stages);
Sys.nb_stages=1;

%%  Solving for the first horizon L
M = Sys.bigM; % big M

% Initialize discrete data for the controller and environment

donen = zeros(1,2*L-1); % done(1:k) = 1 iff everything has been computed up to step k
pn = -1*M*ones(1,L);    % for activating robustness constraints
Un = zeros(nu,2*L-1);
Xn = zeros(max(nx,1),2*L);
if (nx>0)
    Xn(:,1) = x0;           % only X0 is already computed
end
pn(1) = rob;

Upred = zeros(nu,2*L-1);
Xpred = zeros(nx,2*L);

u_new = [];
w_new = [];
x_new = [];
y_new = [];
time_new = 0;
sys_time=[];
params = {};

% call solver for the first horizon
i_transient=1;
i_past = 1;

%% Init system and model data
Sys.system_data=struct;
Sys.model_data=struct;

Sys.system_data.time = [];
Sys.system_data.U = [];
Sys.system_data.X = x0;
Sys.system_data.Y = [];
Sys.system_data.W = [];

Sys.model_data.time = Sys.time_d;
Sys.model_data.X = repmat(0*Sys.time_d(1:end), [nx 1]);
Sys.model_data.Y = repmat(0*Sys.time_d(1:end-1), [ny 1]);
Sys.model_data.U = repmat(0*Sys.time_d(1:end-1), [nu 1]);
Sys.model_data.W = Wn;

compute_input();
        
u_new = Upred(:,i_transient);
w_new = Wn(:,i_transient);
%[x_new, y_new] = system_step(Sys, x0, u_new, w_new);
[x_new, y_new] = apply_input();
i_past =  i_past+1;
params{end+1} = {i_transient,Sys.time_d,donen,pn,Xn,Un,Wn};

update_hist_data();
Sys = update_plot(Sys);

%% loop
%pause
i_transient = i_transient+1;
while (Sys.time_d(end)+ts< time(end))
    pause;
    x0 = x_new(:,end);
    time_new = time_new+ts;
    
    %% updates the model of controller and environment for the next horizon
    update_controller_data();
    
    %% compute input for the next horizon
    compute_input();
    
    %% update state        
    u_new = Upred(:,i_transient);
    w_new = Wn(:,i_transient);
    [x_new, y_new] = apply_input();
   
    sys_time = sys_time(2:end);
    x_new = x_new(:,2:end);
    y_new = y_new(:,2:end);
    u_new = u_new(:,2:end);
    w_new = w_new(:,2:end);
    
    i_past =  i_past+1;
    params{end+1} = {i_transient,Sys.time_d,donen,pn,Xn,Un,Wn};

    %% Update plots
    update_hist_data();
    Sys= update_plot(Sys);
    
    if i_transient < L
        i_transient = i_transient+1;
    end
    drawnow;
    %   pause;
    if StopRequest
        break;
    end
end

    function compute_input()
        [sol_control, errorflag1] = controller{{donen,pn,Xn,Un,Wn}};
        if(errorflag1==0)  % found a good control
            %disp(['Yalmip: ' yalmiperror(errorflag1)])
            Upred = sol_control{1};
            Xpred = sol_control{2};
        elseif (errorflag1==1 || errorflag1==15||errorflag1==12)  % some error, infeasibility or else
            disp(['Yalmip error (disturbance too bad ?): ' yalmiperror(errorflag1)]); % probably there is no controller for this w
            StopRequest=1;
        else
            disp(['Yalmip error: ' yalmiperror(errorflag1)]); % some other error
        end        
    end

    function update_hist_data()
        
        Sys.system_data.time = [Sys.system_data.time sys_time];
        Sys.system_data.U = [Sys.system_data.U u_new];
        Sys.system_data.X =  [Sys.system_data.X x_new];
        Sys.system_data.Y = [Sys.system_data.Y y_new];
        Sys.system_data.W  =[ Sys.system_data.W   w_new ];
        
        Sys.model_data.time = Sys.time_d;
        Sys.model_data.X = double(Xpred);
        Sys.model_data.Y = [Sys.sysd.C*double(Xpred(:,1:end-1)) + Sys.sysd.D*[double(Upred); double(Wn(:,1:end-1))]];
        Sys.model_data.U = double(Upred);
        Sys.model_data.W = Wn;
        
    end

    function update_controller_data()
        
        if (i_past>= L+1)
            Sys.time_d = Sys.time_d+ts; % move forward one time step
        end
        
        if i_transient<L
            donen(1:i_transient-1) = 1;  % we reuse what has been computed at the previous step
            Un(:,1:i_transient-1) = Sys.system_data.U(:,1:i_transient-1); %  previously computed inputs
            Xn(:,1:i_transient) = Sys.system_data.X(:, 1:i_transient);
            pn(i_transient) = rob;
        else
            pn(1:L) = rob*ones(1,L);
            donen(1:L-1) = 1;
            Un(:,1:L-1) = Sys.system_data.U(:,end-L+2:end);     %    previously computed inputs
            Xn(:,1:L) =   Sys.system_data.X(:,end-L+1:end);     %    previously computed temperatures
        end
        
        Wn = sensing(Sys);
        
    end

    function [x_new, y_new] = apply_input()
     
        sys_time = time(time>time_new+10*eps);
        sys_time = sys_time(sys_time< time_new+ts-10*eps);
        sys_time = [time_new sys_time time_new+ts];
        w_new = interp1(time, Sys.Wref', sys_time);
        u_new = repmat(u_new,1, size(w_new,2));
        [x_new, y_new] = system_sim(Sys,u_new, w_new, sys_time,x0);
        
        
    end


end

function Stop()
global StopRequest;
StopRequest = true;
end
