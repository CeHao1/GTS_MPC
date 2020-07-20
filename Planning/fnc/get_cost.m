function [J,beta,an]=get_cost(sum_out)
w_kap=1.2e1;
w_delta=1e1;
w_Vy=0.015;

xout=sum_out{1,end};
uout=sum_out{2,end};
Vx=xout(1,:);
Vy=xout(2,:);
dpsi=xout(3,:);
dsteer=extend(uout(1,:));
V=sqrt(Vx.^2+Vy.^2);
% J=w_kap*(dpsi./V).^2+w_Vy*Vy.^2+w_delta*dsteer.^2;
J=V;

% J=w_kap*(dpsi./V).^2;
% J=w_Vy*Vy.^2;

% J=w_delta*dsteer.^2;
beta=Vy./Vx;


%%
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
Dd=(Fyf.*sin(delta))/m-dpsi.*Vy;

w_wind=2.7e-4;
a_wind=w_wind*Vx.^2;

an=dpsi.*Vy;



end