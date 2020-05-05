function y=car_linear(x,u)
dt=1/60;

m=1060;
Iz=1493.4;
lf=1.0462;
lr=1.2638;

a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;

Ns=6;
Nc=2;

%%
Vx=x(1);
Vy=x(2);
dpsi=x(3);
epsi=x(4);
ey=x(5);
delta=x(6);

kap=0;
x0=x;

%% linear

alpf=delta-atan2(Vy+lf*dpsi,Vx);
alpr=-atan2(Vy-lr*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Fyr=a21*tanh(a22*alpr);
Kf=a11*a12*(1-tanh(a12*alpf)^2);
Kr=a21*a22*(1-tanh(a22*alpr)^2);
bf=Fyf-Kf*alpf;
br=Fyr-Kr*alpr;

A22=-(Kf+Kr)/(m*Vx);
A23=-Vx-(Kf*lf-Kr*lr)/(m*Vx);
A32=-(Kf*lf-Kr*lr)/(Iz*Vx);
A33=-(Kf*lf^2+Kr*lr^2)/(Iz*Vx);
B26=Kf/m;
B36=Kf*lf/Iz;
D1=-(Fyf*sin(delta))/m+dpsi*Vy;
D2=(bf+br)/m;
D3=(lf*bf-lr*br)/Iz;

den=1-kap*ey;
R41=-cos(epsi)*kap/den;
R42=sin(epsi)*kap/den;
R44=(Vx*sin(epsi)+Vy*cos(epsi))*kap/den;
R45=(Vx*cos(epsi)-Vy*sin(epsi))*kap^2/den^2;
R51=sin(epsi);
R52=cos(epsi);
R54=Vx*cos(epsi)-Vy*sin(epsi);

partial_epsi=[R41,R42,1,R44,R45,0,0];
partial_ey=[R51,R52,0,R54,0,0,0];

depsi=dpsi-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
dey=Vx*sin(epsi)+Vy*cos(epsi);

D4=depsi-partial_epsi*x0;
D5=dey-partial_ey*x0;
%  D4=-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
% D5=dey-partial_ey*x0+R51*Vx;

    Ac=[[0,0,0,0,0,0,0];
        [0,A22,A23,0,0,B26,0];
        [0,A32,A33,0,0,B36,0];
        partial_epsi;
        partial_ey;
        [0,0,0,0,0,0,0];
        [0,0,1,0,0,0,0]];
    Bc=[[1,0];
        [0,0];
        [0,0];
        [0,0];
        [0,0];
        [0,1]
        [0,0]];
    Dc=[D1,D2,D3,D4,D5,0,0]';
    
    Ad=Ac*dt+eye(Ns);
    Bd=Bc*dt;
    Dd=Dc*dt;
      
    y=Ad*x+Bd*u+Dd;

end