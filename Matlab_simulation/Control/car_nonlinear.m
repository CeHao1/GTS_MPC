function y=car_nonlinear(x,u)
dt=1/60;
% X,Y,psi,Vx,Vy,dpsi
psi=x(3);
Vx=x(4);
Vy=x(5);
dpsi=x(6);

a=u(1);
delta=u(2);


m=1060;
Iz=1493.4;
lf=1.0462;
lr=1.2638;

a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;
%% nonlinear

a_d=a;

alpf=delta-atan2(Vy+lf*dpsi,Vx);
alpr=-atan2(Vy-lr*dpsi,Vx);

Fyf=a11*tanh(a12*alpf);
Fyr=a21*tanh(a22*alpr);

dx(1)=Vx*cos(psi)+Vy*sin(psi);                           % X
dx(2)=Vx*sin(psi)-Vy*cos(psi);                           % Y
dx(3)=dpsi;                                                       % psi     
dx(4)=a_d-(Fyf*sin(delta))/m+dpsi*Vy;              % Vx
dx(5)=(Fyf*cos(delta)+Fyr)/m-dpsi*Vx;             % Vy
dx(6)=(lf*Fyf*cos(delta)-lr*Fyr)/Iz;                    %dpsi

y=x+dx*dt;


end