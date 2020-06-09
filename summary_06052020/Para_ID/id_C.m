function [C,Izz] = id_C(d, params, plotflag)
    m = params.mass;
    
%     cvx_begin
%         variable C
%         J=d.ay*m/2-C*(d.saf+d.sar);
%         minimize(norm(J))
%     cvx_end
    
    myfunc = @(C,xdata) C*xdata;
    xdata = (d.saf+d.sar);
    ydata = d.ay*m/2;
    [C,RESNORM,RESIDUAL] = lsqcurvefit(myfunc, 1, xdata, ydata);

    if plotflag
        figure()
        hold on
        plot((d.saf + d.sar),d.ay*m/2,'b.')
        plot((d.saf+d.sar),(d.saf+d.sar)*C,'k','linewidth',1.5)
        title(strcat('C_\alpha : ',num2str(C)),'fontsize',15)
        xlabel('\alpha','fontsize',15)
        ylabel('Fy','fontsize',15)
end