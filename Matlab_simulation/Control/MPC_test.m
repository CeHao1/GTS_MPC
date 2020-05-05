close all




Vx=xout(1:Ns:(Np+1)*Ns);
Vy=xout(2:Ns:(Np+1)*Ns);
dpsi=xout(3:Ns:(Np+1)*Ns);
epsi=xout(4:Ns:(Np+1)*Ns);
ey=xout(5:Ns:(Np+1)*Ns);
delta=xout(6:Ns:(Np+1)*Ns);
acc=xout(1+(Np+1)*Ns:Nc:(Np+1)*(Ns+Nc));
d_delta=xout(2+(Np+1)*Ns:Nc:(Np+1)*(Ns+Nc));
%%
% x0=[Vx,u(5),u(6),epsi,ey,u(7)]';

yn(1:6,1)=u(1:6,1);
yl(1:3,1)=u(4:6,1);
control=[acc(2:end),delta(1:end-1)]';

for i=1:Np
    
    yn(:,i+1)=car2(yn(:,i),control(:,i));
%     yl(:,i+1)=car3(yl(:,i),control(:,i));
    
end

yl=car4(yl(:,1),control);

Vx2=yn(4,:);
Vy2=yn(5,:);
dpsi2=yn(6,:);
beta2=Vy2./Vx2;

Vx3=yl(1,:);
Vy3=yl(2,:);
dpsi3=yl(3,:);
beta3=Vy3./Vx3;
%%
figure
subplot(241)
hold on
plot(Vx)
plot(Vx2,'r')
plot(Vx3,'k')
hold off
title('Vx','fontsize',12)

subplot(242)
hold on
plot(Vy)
plot(Vy2,'r')
plot(Vy3,'k')
hold off
title('Vy','fontsize',12)

subplot(243)
hold on
plot(dpsi)
plot(dpsi2,'r')
plot(dpsi3,'k')
hold off
title('d\psi','fontsize',12)

subplot(244)
hold on
plot(epsi)
hold off
title('e\psi','fontsize',12)

subplot(245)
hold on
plot(ey)
hold off
title('ey','fontsize',12)

subplot(246)
hold on
plot(delta)
hold off
title('\delta','fontsize',12)

beta=Vy./Vx;
beta_epsi=beta+epsi;
dey=Vy+Vx.*epsi;

subplot(247)
hold on
plot(beta_epsi,'b')
plot(dey,'r')
hold off
title('beta+epsi','fontsize',12)


beta_epsi2=beta2+epsi';
beta_epsi3=beta3+epsi';

subplot(248)
hold on
plot(beta_epsi,'b')
plot(beta_epsi2,'r')
plot(beta_epsi3,'k')
hold off
title('beta+ epsi1 & 2')






