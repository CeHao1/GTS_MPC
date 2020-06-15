close all

mu=1.35;
g=9.8;
dt=1/60;

m=1060;
Iz=1493.4;
lf=1.0462;
lr=1.2638;

a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;

Vx=xout(1,:);
Vy=xout(2,:);
dpsi=xout(3,:);
delta=xout(6,:);

alpf=delta-atan2(Vy+lf*dpsi,Vx);
alpr=-atan2(Vy-lr*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Fyr=a21*tanh(a22*alpr);
% Kf=a11*a12*(1-tanh(a12*alpf)^2);
% Kr=a21*a22*(1-tanh(a22*alpr)^2);
% bf=Fyf-Kf*alpf;
% br=Fyr-Kr*alpr;

dVy=extend(diff(Vy)/dt);
ay=(Fyf+Fyr)/m;
ay2=dVy+Vx.*dpsi;


figure
subplot(221)
hold on
plot(ay,'b')
plot(ay2,'r')
hold off

subplot(222)
hold on
plot(-Vy./Vx,'b')
plot(-dpsi*lf./Vx,'r')
plot(delta,'k')
hold off

subplot(223)
hold on
plot(delta,dVy,'b.')
plot(delta,Vx.*dpsi,'r.')
hold off

subplot(224)
plot(delta,Vy)
%%
figure
subplot(221)
hold on
plot(alpf,'b');
plot(alpr,'r');
hold off
title('alpha f and r')

subplot(222)
hold on
plot(Fyf,'b')
plot(Fyr,'r')
hold off
title('Fy f and r')

subplot(223)
hold on
plot(ones(1,length(xout))*mu*g)
plot(-ones(1,length(xout))*mu*g)
plot((Fyf.*cos(delta)+Fyr)/m,'r')
% plot((Fyf+Fyr)/m,'r')
hold off
title('\mu g , (Fyf*cos(delta)+Fyr)/m')


