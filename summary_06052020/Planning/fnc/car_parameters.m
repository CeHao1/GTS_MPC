

classdef car_parameters
    properties
        %% car parameters
        mu = 1.35;          % coef of friction
        g = 9.81;           % acceleration of gravity
        a11 = 4800*2;       % tanh model constant
        a21 = 3720*2;       % tanh model constant
        a12 = 7.0;          % tanh model constant
        a22 = 10.3;         % tanh model constant
        m = 1060;           % mass
        a_e = 1.3;          % engine limit
        a_b = -9.5;         % braking limit
        lf = 1.0462;        % front length
        lr = 1.2638;        % rear length
        w_wind = 2.7e-4;    % wind coefficient
        Iz = 1493.4;        % moment of inertia
        
        %% track planning parameters
        gap = 0.5; % track side gap
        dsteer_limit = 50; % limit on derivative of steering angle
        steerlimit = 10;
        trackwidth = 3.75;     
    end % end properties
    methods 
        function car_parameters(self, carcode)
            switch carcode 
                case 3383 % demio
                    self.mu = 1.5;          % coef of friction
                    self.a11 = 4800*2;       % tanh model constant
                    self.a21 = 3720*2;       % tanh model constant
                    self.a12 = 7.0;          % tanh model constant
                    self.a22 = 10.3;         % tanh model constant
                    self.m = 1355.2;           % mass
                    self.a_e = 1.3;          % engine limit
                    self.a_b = -9.5;         % braking limit
                    self.lf = 1.2928996;     % front length (given by api: 1.69051)
                    self.lr = 2.028624046;   % rear length (given by api: 2.35349)
                    self.w_wind = 2.7e-4;    % wind coefficient
                    self.Iz = 1493.4;        % moment of inertia
                case 2148 % roadster
                    self.mu = 1.35;          % coef of friction
                    self.a11 = 4800*2;       % tanh model constant
                    self.a21 = 3720*2;       % tanh model constant
                    self.a12 = 7.0;          % tanh model constant
                    self.a22 = 10.3;         % tanh model constant
                    self.m = 1060;           % mass
                    self.a_e = 1.3;          % engine limit
                    self.a_b = -9.5;         % braking limit
                    self.lf = 1.0462;        % front length
                    self.lr = 1.2638;        % rear length
                    self.w_wind = 2.7e-4;    % wind coefficient
                    self.Iz = 1493.4;        % moment of inertia
            end
        end % end initialization function
    end % end methods
                
            
                
end