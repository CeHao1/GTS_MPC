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

Vx=xout(1,:);
Vy=xout(2,:);
dpsi=xout(3,:);

ay_fce=(Fyf.*cos(delta)+Fyr)/m;

dt_ds=ds_new./Vx;
dVy=extend(diff(Vy))./dt_ds;
ay_dyna=dVy+Vx.*dpsi;

figure
hold on
plot(ones(1,length(xout))*mu*g)
plot(-ones(1,length(xout))*mu*g)
plot(ay_fce,'b')
plot(ay_dyna,'r')

hold off



