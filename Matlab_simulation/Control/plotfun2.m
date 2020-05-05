
close all
addpath('./map')
name1p='centerline1_demo.csv';
name2p='centerline2_tokyo.csv';
map_data=csvread(name2p);


%%
lap_start=find(abs(diff(sum_s))>10);
range=lap_start(1)+1:lap_start(2);
laptime=t(lap_start(2)+1)-t(lap_start(1)+1);
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
plot(map_data(:,1),map_data(:,2),'b--');
plot(map_data(:,3),map_data(:,4),'r--');
plot(map_data(:,5),map_data(:,6),'m--');
h1=plot(reference(:,2),reference(:,3),'r','linewidth',2);
h2=plot(X(range),Y(range),'k','linewidth',2);
plot(X(range(1)),Y(range(1)),'bo','linewidth',2);


hold off
axis equal
plotdist=100;
axis([min(X)-plotdist,max(X)+plotdist,min(Y)-plotdist,max(Y)+plotdist]);
title('X-Y position','fontsize',15)
legend([h1,h2],'reference','real path')


%%
figure
subplot(2,3,1)
plot(t(range),input(range,1))
title('ax','fontsize',15)

% steer=smooth(input(:,2),1000);
steer=input(:,2);
subplot(2,3,2)
plot(t(range),steer(range)/pi*180);
title('steer degree','fontsize',15)

subplot(233)
plot(t(range),sum_s(range))
title(strcat('s, laptime: ',num2str(laptime)),'fontsize',15)

alpf=steer-(Vy+lf*dpsi)./Vx;
alpr=-(Vy-lr*dpsi)./Vx;

Fyf=a11*tanh(a12*alpf);
Fyr=a21*tanh(a22*alpr);
ay=(Fyf.*cos(steer)+Fyr)/m;

subplot(234)
% hold on
% plot(t(range),alpf(range)/pi*180,'b')
% plot(t(range),alpr(range)/pi*180,'r')
% hold off
% title('alpf,alpr  degree','fontsize',15)

hold on
plot(ones(1,length(range))*mu*g,'b')
plot(-ones(1,length(range))*mu*g,'b')
plot(ay(range),'r')
hold off
title('\mu g , ay')


dey=Vy+Vx.*epsi;
subplot(235)
plot(t(range),dey(range))
title('dey','fontsize',15)

psi_ref=spline(reference(:,1),reference(:,6),sum_s);
indx2=(psi>max(psi_ref));
psi(indx2)=psi(indx2)-2*pi;
psi=psi+2*pi;
subplot(236)
hold on
plot(t(range),psi(range),'r')
plot(t(range),psi_ref(range),'b')
hold off
title('psi')

%%

figure
kap=spline(reference(:,1),reference(:,5),sum_s);
subplot(2,3,1);
plot(t(range),kap(range))
title('kap','fontsize',15)

Vx_ref=spline(reference(:,1),reference(:,4),sum_s);
subplot(232)
dpsi_ref=Vx_ref.*kap;
hold on
plot(t(range),dpsi(range),'b');
plot(t(range),dpsi_ref(range),'r','linewidth',1.5)
title('d\psi','fontsize',15)

beta=(Vy./Vx);
epsi_beta=epsi+beta;
subplot(2,3,3)
hold on
plot(t(range),beta(range),'r');
plot(t(range),epsi(range),'b');
plot(t(range),epsi_beta(range),'k','linewidth',2)
hold off
title('e\psi, \beta, e\psi+\beta','fontsize',15)

subplot(2,3,6)
plot(t(range),ey(range))
title('ey','fontsize',15)

subplot(2,3,4)
hold on
plot(t(range),Vx_ref(range),'b')
plot(t(range),Vx(range),'r')
hold off
title('Vx','fontsize',15)
legend('ref','simu')

subplot(2,3,5)
plot(t(range),Vy(range))
title('Vy','fontsize',15)




% figure
% hold on
% plot(ay(range),'b')
% plot(ay2(range),'r')
% 
% hold off


