function [a11,a12,a21,a22] = id_tanh(d, p, pID, plotflag)
    Izc = pID.Izz;
    m = p.mass;
    L = p.wheel_base;

    psi=d.rt2;
    
    % force on front/back wheel
    Fyf=(pID.lr*d.ay*m+Izc*(d.d2psi))/L/2; % each wheel
    Fyr=(pID.lf*d.ay*m-Izc*(d.d2psi))/L/2;
    
    % tanh function
    myfunc = @(b1,x) b1(1)*tanh(b1(2)*x);

    % front tire
    a_tanh_f = nlinfit(d.saf,Fyf,myfunc,[10000,20]);
    if plotflag
        plot_tanh(d.saf,Fyf,a_tanh_f);
    end
    
    % rear tire
    a_tanh_r = nlinfit(d.sar,Fyr,myfunc,[10000,20]);
    if plotflag 
        plot_tanh(d.sar,Fyr,a_tanh_f);
    end
    
    
    % save variables
    a11 = a_tanh_f(1);
    a12 = a_tanh_f(2);
    a21 = a_tanh_r(1);
    a22 = a_tanh_r(2);

end