function [F,Plow,Pup] = STL2MILP_robust_interval(phi,k,ts,var,M)
% STL2MILP_robust_interval  constructs MILP constraints in YALMIP that compute
%                           the robust interval of satisfaction, i.e. the 
%                           lower and upper bounds on the robustness of 
%                           satisfaction for specification phi
%
%
% Input: 
%       phi:    an STLformula
%       k:      the length of the trajectory
%       ts:     the interval (in seconds) used for discretizing time
%       var:    a dictionary mapping strings to variables
%       M:   	a large positive constant used for big-M constraints  
%
% Output: 
%       F:  YALMIP constraints
%       P:  a struct containing YALMIP decision variables representing 
%           upper (Plow) and lower (Pup) bounds on quantitative satisfaction 
%           of phi over each time step from 1 to k 
%
% :copyright: TBD
% :license: TBD

% NOT IN WORKING CONDITION

    if (nargin==4);
        M = 1000;
    end;
        
    F = [];
    Plow = [];
    Pup = [];
    
    if ischar(phi.interval)
        interval = [str2num(phi.interval)];
    else
        interval = phi.interval;
    end
    
    a = interval(1);
    b = interval(2);
    
    if a == Inf
        a = k*ts;
    end
    if b == Inf
        b = k*ts;
    end
    
    a = max([0 floor(a/ts)-1]); 
    b = ceil(b/ts)-1; 
    
    switch (phi.type)
        
        case 'predicate'
            [F,Plow,Pup] = pred(phi.st,k,var,M);
                     
        case 'not'
            [Frest,Prest1,Prest2] = STL2MILP_robust_interval(phi.phi,k,ts, var,M);
            [Fnot, Pnot1,Pnot2] = not(Prest1,Prest2);
            F = [F, Fnot, Frest];
            Plow = Pnot1;
            Pup = Pnot2;

        case 'or'
            [Fdis1,Pdis11,Pdis12] = STL2MILP_robust_interval(phi.phi1,k,ts, var,M);
            [Fdis2,Pdis21,Pdis22] = STL2MILP_robust_interval(phi.phi2,k,ts, var,M);
            [For, Por1,Por2] = or([Pdis11;Pdis21],[Pdis12;Pdis22],M);
            F = [F, For, Fdis1, Fdis2];
            Plow = Por1;
            Pup = Por2;

        case 'and'
            [Fcon1,Pcon11,Pcon12] = STL2MILP_robust_interval(phi.phi1,k,ts, var,M);
            [Fcon2,Pcon21,Pcon22] = STL2MILP_robust_interval(phi.phi2,k,ts, var,M);
            [Fand,Pand1,Pand2] = and([Pcon11;Pcon21],[Pcon12;Pcon22],M);
            F = [F, Fand, Fcon1, Fcon2];
            Plow = Pand1;
            Pup = Pand2;

        case '=>'
            [Fant,Pant1,Pant2] = STL2MILP_robust_interval(phi.phi1,k, ts,var,M);
            [Fcons,Pcons1,Pcons2] = STL2MILP_robust_interval(phi.phi2,k,ts, var,M);
            [Fnotant,Pnotant1,Pnotant2] = not(Pant1,Pant2);
            [Fimp,PimPlow,PimPup] = or([Pnotant1;Pcons1],[Pnotant2;Pcons2],M);
            F = [F, Fant, Fnotant, Fcons, Fimp];
            Plow = PimPlow;
            Pup = PimPup;
            
        case 'always'
            [Frest,Prest1,Prest2] = STL2MILP_robust_interval(phi.phi,k, ts, var,M);
            [Falw,Palw1,Palw2] = always(Prest1,Prest2,a,b,k,M);
            F = [F, Falw];
            F = [F, Frest];
            Plow = Palw1;
            Pup = Palw2;

        case 'eventually'
            [Frest,Prest1,Prest2] = STL2MILP_robust_interval(phi.phi,k, ts, var,M);
            [Fev,Pev1,Pev2] = eventually(Prest1,Prest2,a,b,k,M);
            F = [F, Fev];
            F = [F, Frest];
            Plow = Pev1;
            Pup = Pev2;
          
        case 'until'
            [Fp,PPlow,PPup] = STL2MILP_robust_interval(phi.phi1,k, ts, var,M);
            [Fq,Pq1,Pq2] = STL2MILP_robust_interval(phi.phi2,k, ts, var,M);
            [Funtil,Puntil1,Puntil2] = until(PPlow,PPup,Pq1,Pq2,a,b,k,M);
            F = [F,Funtil,Fp,Fq];
            Plow = Puntil1;
            Pup = Puntil2;
    end   
end

function [F,z1,z2] = pred(st,k,var,M)
    % Enforce constraints based on predicates 
    % 
    % var is the variable dictionary    
        
    fnames = fieldnames(var);
    
    for ifield= 1:numel(fnames)
        eval([ fnames{ifield} '= var.' fnames{ifield} ';']); 
    end          
        
    
    st = regexprep(st,'\[t\]','\(t\)'); % Breach compatibility ?
    if findstr('<', st)
        st = regexprep(st,'<','<= ');
        st = regexprep(st,'<= ',' +');
        st = ['-',st];
    end
    if findstr('>', st)
        st = regexprep(st,'>','>= ');
        st = regexprep(st,'>= ','-');
    end
         
    F = [];
    
    zAll = [];
    
    for l=1:k
        % the below conditional statements allow specifications to refer to
        % the previous and next time steps (e.g. when controlling input)
        t_st = st;
        if l<k
            t_st = regexprep(t_st,'t+1\)',[num2str(l+1) '\)']);
        else
            t_st = regexprep(t_st,'t+1\)',[num2str(l) '\)']);
        end
        if l>1
            t_st = regexprep(t_st,'t-1\)',[num2str(l-1) '\)']);
        else
            t_st = regexprep(t_st,'t-1\)',[num2str(l) '\)']);
        end
        t_st = regexprep(t_st,'t\)',[num2str(l) '\)']);
        
        try %upper and lower bounds
            z = eval(t_st);
        end
        
%         z1Temp = sdpvar(size(val));
%         z2Temp = sdpvar(size(val));
%         F = [F, z1Temp >= val];
%         F = [F, z2Temp <= val];
%         zAll1 = [zAll1,z1Temp];
%         zAll2 = [zAll2,z2Temp];

        zAll = [zAll, z];

    end
    
    
    % take the and over all dimensions for multi-dimensional signals
    z1 = sdpvar(1,k);
    z2 = sdpvar(1,k);
    
    for i=1:k
        [Fnew, z1(:,i), z2(:,i)] = and(zAll(:,i),zAll(:,i),M);
        F = [F, Fnew];
    end
end

% BOOLEAN OPERATIONS

function [F,Plow,Pup] = and(p_list1,p_list2,M)
    [F,Plow,Pup] = min_r(p_list1,p_list2,M);
end


function [F,Plow,Pup] = or(p_list1,p_list2,M)
     [F,Plow,Pup] = max_r(p_list1,p_list2,M);
end


function [F,Plow,Pup] = not(p_list1,p_list2)
    k = size(p_list1,2);
    m = size(p_list1,1);
    assert( m == 1 )
    Plow = sdpvar(1,k);
    Pup = sdpvar(1,k);
    F = [Plow == -p_list2, Pup == -p_list1];
end



% TEMPORAL OPERATIONS

function [F,P_alw1,P_alw2] = always(Plow,Pup,a,b, k,M)
    F = [];
    P_alw1 = sdpvar(1,k);
    P_alw2 = sdpvar(1,k);
    
    for i = 1:k
        [ia, ib, over] = getIndices(i,a,b,k);
        [F0,P0low,P0up] = and(Plow(ia:ib)',Pup(ia:ib)',M);
        if over
            F0 = [F0, P_alw1(i) == -M];
        else
            F0 = [F0, P_alw1(i)==P0low];
        end
    
        F = [F;F0,P_alw2(i)==P0up];
    end
    
end


function [F,P_ev1,P_ev2] = eventually(Plow,Pup,a,b, k,M)
    F = [];
    P_ev1 = sdpvar(1,k);
    P_ev2 = sdpvar(1,k);
    
    for i = 1:k
        [ia, ib, over] = getIndices(i,a,b,k);
        [F0,P0low,P0up] = or(Plow(ia:ib)',Pup(ia:ib)',M);
        if over
            F0 = [F0, P_ev2(i) == M];
        else
            F0 = [F0, P_ev2(i)==P0up];
        end
    
        F = [F;F0,P_ev1(i)==P0low];
    end
    
end


function [F,P_until1,P_until2] = until(PPlow,PPup,Pq1,Pq2,a,b,k,M)
    
    F = [];
    P_until1 = sdpvar(1,k);
    P_until2 = sdpvar(1,k);
    
    for i = 1:k
        [ia, ib] = getIndices(i,a,b,k);
        F0 = []; 
        P0low = [];
        P0up = [];
        for j = ia:ib
            [F1,Plow1,Plow2] = until_mins(i,j,PPlow,PPup,Pq1,Pq2,M);
            F0 = [F0, F1];
            P0low = [P0low,Plow1];
            P0up = [P0up,Plow2];
        end
        [F4,P41,P42] = max_r(P0low,P0up);
        F = [F;F0,F4,P_until1(i)==P41,P_until2(i)==P42];
    end
    
end


% UTILITY FUNCTIONS

function [F,Plow,Pup] = min_r(p_list1,p_list2,M)
    
    k = size(p_list1,2);
    m = size(p_list1,1);
    
    Plow = sdpvar(1,k);
    z1 = binvar(m,k);
    
    Pup = sdpvar(1,k);
    z2 = binvar(m,k);
     
    F = [sum(z1,1) == ones(1,k),sum(z2,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, Plow(t) <= p_list1(i,t)];     
            F = [F, p_list1(i,t) - (1-z1(i,t))*M <= Plow(t) <= p_list1(i,t) + (1-z1(i,t))*M];
            F = [F, Plow(t) <= p_list1(i,t)];     
            F = [F, p_list1(i,t) - (1-z1(i,t))*M <= Plow(t) <= p_list1(i,t) + (1-z1(i,t))*M];
        
            F = [F, Pup(t) <= p_list2(i,t)];     
            F = [F, p_list2(i,t) - (1-z2(i,t))*M <= Pup(t) <= p_list2(i,t) + (1-z2(i,t))*M];
            F = [F, Pup(t) <= p_list2(i,t)];     
            F = [F, p_list2(i,t) - (1-z2(i,t))*M <= Pup(t) <= p_list2(i,t) + (1-z2(i,t))*M];
        
        end
    end
end

function [F,Plow,Pup] = max_r(p_list1,p_list2,M)

    k = size(p_list1,2);
    m = size(p_list1,1);
    
    Plow = sdpvar(1,k);
    z1 = binvar(m,k);
    
    Pup = sdpvar(1,k);
    z2 = binvar(m,k);
    
    F = [sum(z1,1) == ones(1,k),sum(z2,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, Plow(t) >= p_list1(i,t)];     
            F = [F, p_list1(i,t) - (1-z1(i,t))*M <= Plow(t) <= p_list1(i,t) + (1-z1(i,t))*M];
            
            F = [F, Pup(t) >= p_list2(i,t)];     
            F = [F, p_list2(i,t) - (1-z2(i,t))*M <= Pup(t) <= p_list2(i,t) + (1-z2(i,t))*M];        
        end
    end
end

function [F,Plow,Pup] = until_mins(i,j,PPlow,PPup,Pq1,Pq2,M)
    [F0,P0low,P0up] = min_r(PPlow(i:j)',PPup(i:j)',M);
    [F1,Plow,Pup] = min_r([Pq1(j),P0low],[Pq2(j),P0up],M);
    F = [F0,F1];
end

function [ia, ib, over] = getIndices(i,a,b,k)
    ia = min(k,i+a);
    ib = min(k,i+b);
    if k < i + b
        over = 1;
    else
        over = 0;
    end
end


    
