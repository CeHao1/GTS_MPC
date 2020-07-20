function [kap,ds_new]=get_kap(pos,xout)

psi=pos(1,:);
Car_ey=xout(5,:);
X=pos(2,:);
Y=pos(3,:);

CarX=X-sin(psi).*Car_ey;
CarY=Y+cos(psi).*Car_ey;

% psi_x=xout(4,:)+psi;
% psi2=atan2(diff(CarY),diff(CarX));

ds_new=sqrt(diff(CarX).^2+diff(CarY).^2);
% kap_new=diff(psi_x)./ds_new;
% kap=[kap_new,kap_new(end)*2-kap_new(end-1)];

ds_new=[ds_new,ds_new(end)*2-ds_new(end-1)];

%%
% dx=extend(diff(CarX));
% dy=extend(diff(CarY));
% d2x=extend(diff(dx));
% d2y=extend(diff(dy));
% kap=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;

%%
% psiC=atan2(diff(CarY),diff(CarX));
% index=(psiC>pi/5);
% lp=length(psiC);
% index(1:round(lp/2))=0;
% psiC(index)=psiC(index)-2*pi;
% psiC=extend(psiC);
% kap=diff(psiC)./ds_new(1:end-1);
% kap=extend(kap);

%%
% kap=xout(3,:)./xout(1,:);
Vx=xout(1,:);
Vy=xout(2,:);
dpsi=xout(3,:);
V=sqrt(Vx.^2+Vy.^2);
beta=asin(Vy./Vx);
kap0=dpsi./V;
add_kap=extend(diff(beta))./ds_new;
kap=kap0+add_kap;


% figure
% hold on
% plot(kap,'b')
% plot(kap2,'r')
% hold off

end

