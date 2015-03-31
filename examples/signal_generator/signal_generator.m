classdef signal_generator < STLC_lti
    % this class implements a "signal generator". It has no dynamics and no
    % disturbance, so that the control synthesis problem is the same as
    % finding an input signal that satisfies a given STL formula
    
    methods
        function  SG = signal_generator(nu, nw)
            A =[];
            B =[];
            C =[];
            Du = eye(nu);
            Dw = eye(nw);
            SG = SG@STLC_lti(A,B,[],C,Du,Dw);
        
        end
        
        function rob = monitor(SG,phi)
             SG.bigM = 1000; 
            SG.h = [];
           SG.stl_list{1} = phi;
           SG.controller = get_controller(SG,'robust');
           SG.min_rob= -SG.bigM;
           [SG,~,rob] = SG.run_open_loop(SG.controller);
           figure;
           hold on;
           plot(SG.model_data.time, SG.model_data.W(:,1:end))
           plot(SG.model_data.time, rob, 'k', 'LineWidth',2);
         
           
        end

        function rob = monitor_interval(SG,phi)
           SG.bigM = 1000; 
           SG.h = [];
           SG.stl_list{1} = phi;
           SG.controller = get_controller(SG,'interval');
           SG.min_rob= -1000;
           [SG,~,rob] = SG.run_open_loop(SG.controller);
           figure;
           hold on;
           plot(SG.model_data.time, SG.model_data.W(:,1:end))
           plot(SG.model_data.time, rob, 'k', 'LineWidth',2);
           
        end

        
    end
    
end