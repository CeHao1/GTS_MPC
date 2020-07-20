close all

% 
Vy=x_last(2,:);
dpsi=x_last(3,:);
delta=x_last(6,:);

lf=1.0462;
a11=4800*2;
a12=7.0;
m=1060;
alpf=delta-atan2(Vy+lf*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Dd=(Fyf.*sin(delta))/m-dpsi.*Vy;

w_wind=2.7e-4;
a_wind=w_wind*Vx.^2;

figure
hold on
plot(Dd,'b')
plot(a_wind,'r')
hold off


