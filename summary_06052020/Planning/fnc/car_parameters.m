

classdef car_parameters
    properties
        g = 9.81;           % acceleration of gravity

        %% car parameters
        mu = 1.35;          % coef of friction
        a11 = 4800*2;       % tanh model constant
        a21 = 3720*2;       % tanh model constant
        a12 = 7.0;          % tanh model constant
        a22 = 10.3;         % tanh model constant
        m = 1060;           % mass
        a_e = 1.3;          % engine limit
        a_b = -8;         % braking limit
        lf = 1.0462;        % front length
        lr = 1.2638;        % rear length
        w_wind = 2.7e-4;    % wind coefficient
        Iz = 1493.4;        % moment of inertia
        
        %% track planning parameters
        gap = 0.5;          % track side gap
        dsteer_limit = 50;  % limit on derivative of steering angle
        steerlimit = 10;
%         trackwidth = 3.75; 
        trackwidth = 7;
    end % end properties
    methods 
        
        function obj = car_parameters(loadfile, filename)
            %% constructor function to construct object
            if loadfile
                d = load(filename);
                
                obj.mu = d.mu;
                obj.m = d.m;
                obj.a11 = d.a11; obj.a12 = d.a12; 
                obj.a21 = d.a21; obj.a22 = d.a22;
                obj.lf = d.lf; obj.lr = d.lr;
                obj.Iz = d.Izz;
            end
            
        end % end initialization function
    end % end methods
                
            
                
end