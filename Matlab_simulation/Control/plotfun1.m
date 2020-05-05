

close all
addpath('./map')
name1p='centerline1_demo.csv';
name2p='centerline2_tokyo.csv';
map_data=csvread(name2p);

%%
% fetch
X=states(:,1);
Y=states(:,2);
psi=states(:,3);
Vx=states(:,4);
Vy=states(:,5);
dpsi=states(:,6);


figure
% subplot(2,3,[1:2,4:5])
hold on
h1=plot(reference(:,2),reference(:,3),'r','linewidth',2);
h2=plot(X,Y,'k','linewidth',2);
plot(map_data(:,1),map_data(:,2),'b--');
plot(map_data(:,3),map_data(:,4),'r--');
plot(map_data(:,5),map_data(:,6),'m--');

hold off
axis equal
plotdist=100;
axis([min(map_data(:,1))-plotdist,max(map_data(:,1))+plotdist,min(map_data(:,2))-plotdist,max(map_data(:,2))+plotdist]);
title('X-Y position','fontsize',15)
legend([h1,h2],'reference','real path')


%%
figure
subplot(2,3,1)
plot(t,input(:,1))
title('a','fontsize',15)

% steer=smooth(input(:,2),1000);
steer=input(:,2);
subplot(2,3,2)
plot(t,steer/pi*180);
title('steer degree','fontsize',15)

subplot(233)
plot(t,sum_s)
title('s','fontsize',15)

alpf=steer-(Vy+lf*dpsi)./Vx;
alpr=-(Vy-lr*dpsi)./Vx;

subplot(234)
hold on
plot(t,alpf/pi*180,'b')
plot(t,alpr/pi*180,'r')
hold off
title('alpf,alpr  degree','fontsize',15)

dey=Vy+Vx.*epsi;
subplot(235)
plot(t,dey)
title('dey','fontsize',15)

psi_ref=spline(reference(:,1),reference(:,6),sum_s);
indx2=(psi>max(psi_ref));
psi(indx2)=psi(indx2)-2*pi;
subplot(236)
hold on
plot(psi,'r')
plot(psi_ref,'b')
hold off
title('\psi','fontsize',15)
%%

figure
kap=spline(reference(:,1),reference(:,5),sum_s);
subplot(2,3,1);
plot(t,kap)
title('kap','fontsize',15)

Vx_ref=spline(reference(:,1),reference(:,4),sum_s);
subplot(232)
% dpsi_ref=Vx_ref.*kap;
dpsi_ref=Vx.*kap;
hold on
plot(t,dpsi,'b');
plot(t,dpsi_ref,'r','linewidth',1.5)
title('d\psi','fontsize',15)

beta_epsi=epsi+(Vy./Vx);
subplot(2,3,3)
hold on
plot(t,(Vy./Vx),'r');
plot(t,epsi,'b');
plot(t,epsi+(Vy./Vx),'k','linewidth',2)
hold off
title('e\psi, \beta, e\psi+\beta','fontsize',15)

subplot(2,3,6)
plot(t,ey)
title('ey','fontsize',15)

subplot(2,3,4)
hold on
plot(t,Vx_ref,'b')
plot(t,Vx,'r')
hold off
title('Vx','fontsize',15)

subplot(2,3,5)
plot(t,Vy)
title('Vy','fontsize',15)


%% 
% psi_center=spline(reference(:,1),reference(:,6),sum_s);
% psi_XY=atan2(diff(Y),diff(X));
% tmp=(psi_XY<-pi/2);
% psi_XY(tmp)=psi_XY(tmp)+2*pi;
% 
% 
% figure
% hold on
% plot(psi_XY,'b')
% plot(psi_center+epsi,'r')
% % plot(psi_center,'k')
% hold off
% 
% legend('psi XY','psi center+ epsi')









