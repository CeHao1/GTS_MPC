function Izz = id_Izz(d, params,paraID,plotflag)
    m = params.mass;
    L = params.wheel_base;
    C = paraID.C;
    xx = 2*d.d2psi/2;
    yy = C*L*(d.saf-d.sar) - d.ay*m*(paraID.lr-paraID.lf)/2;

%     cvx_begin
%         variable Izz
%         J2=xx*Izz-yy
%         minimize(norm(J2))
%     cvx_end
    
    myfunc = @(Izz, xdata) Izz*xdata;
    [Izz,RESNORM,RESIDUAL] = lsqcurvefit(myfunc, 1, xx, yy);
    
    
    if plotflag
        figure()
        hold on
        plot(xx,yy,'b.')
        plot(xx,xx*Izz,'r','linewidth',2)
        hold off
        title(strcat('Izz :',num2str(Izz)),'fontsize',15)
        ylabel('M','fontsize',15)
        
end