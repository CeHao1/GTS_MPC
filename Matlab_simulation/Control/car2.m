function y=car2(x,u)

dt=1/60;
m=1060;
Iz=1493.4;
% Caf=33240*2;
% Car=33240*2;
L=2.3100;
lf=1.0462;
lr=1.2638;

% X=x(1);
% Y=x(2);
psi=x(3);
vx=x(4);
vy=x(5);
dpsi=x(6);

%  input a,delta u(1),u(2)
a=u(1);
delta=u(2);

%  update
alpf=delta-atan2(vy+lf*dpsi,vx);
alpr=-atan2(vy-lr*dpsi,vx);
% Fyf=Caf*alpf;
% Fyr=Car*alpr;

% nonlinear tire 1
% a1=4220;
% a2=8.45;
% Fyf=a1*tanh(a2*alpf);
% Fyr=a2*tanh(a2*alpr);

% nonlinear tire 2
a11=4800;
a12=7.0;
a21=3720;
a22=10.3;
Fyf=a11*tanh(a12*alpf)*2;
Fyr=a21*tanh(a22*alpr)*2;

% confine acc by friction circle
% a_m=sqrt(((a11+a12)^2-(Fyf+Fyr)^2))/m;
% if abs(a)>a_m
%     a_d=a_m*sign(a);
% else
    a_d=a;
% end
w_wind=2.7e-4;
a_wind=w_wind*vx.^2;


dx(1)=vx*cos(psi)-vy*sin(psi);                           % X
dx(2)=vx*sin(psi)+vy*cos(psi);                           % Y
dx(3)=dpsi;                                                       % psi     
dx(4)=a_d-(Fyf*sin(delta))/m+dpsi*vy-a_wind;      % vx
dx(5)=(Fyf*cos(delta)+Fyr)/m-dpsi*vx;             % vy
dx(6)=(lf*Fyf*cos(delta)-lr*Fyr)/Iz;                    % dpsi

y=x+dt*dx';


end



