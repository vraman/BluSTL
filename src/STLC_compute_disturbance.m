function [Sys, status] = STLC_compute_disturbance(Sys, adversary)
% STLC_compute_disturbance
%
% Input:
%       Sys: an STLC_lti instance
%       controller: a YALMIP optimizer object representing the system's
%                   optimization problem for adversary
%
% Output:
%       Sys: modified with additional model_data
%       params: controller data
%
% :copyright: TBD
% :license: TBD

% Assumes compute_input has been called before, hence we reuse the
% model_data fields.

donen =  Sys.model_data.done;
pn    =  Sys.model_data.p;
Xpred = Sys.model_data.X; 
Upred = Sys.model_data.U;
Wrefn = sensing(Sys);  % so far, returns the discretized Wref...  

%% call solver
[sol_adv, errorflag2] = adversary{{donen,pn,Xpred,Upred,Wrefn}};
Wn = sol_adv{1}
wcrob = sol_adv{3}
           
if(errorflag2==0 && wcrob(1) <= 0)  % found a bad disturbance
    disp(['Yalmip: ' 'Found a bad disturbance'])
    Sys.model_data.Wbad =Wn;
elseif(errorflag2==0 && wcrob(1) > 0) % no disturbance can violate specs
    fprintf('Control is good\n');
    Sys.model_data.Wbad =[];
elseif (errorflag2==true || errorflag2==15||errorflag2==12)  % some error, infeasibility or else
    disp(['Yalmip error 1: ' yalmiperror(errorflag2)]);
else
    disp(['Yalmip error 2: ' yalmiperror(errorflag2)]); % some other error
end

status = errorflag2;

Sys.model_data.W =Wn;
Sys.model_data.rob = wcrob;
Sys.model_data.X = double(sol_adv{2});
Sys.model_data.Y = [Sys.sysd.C*Sys.model_data.X(:,1:end-1) + Sys.sysd.D*[Sys.model_data.U; double(Wn(:,1:end-1))]];



end


