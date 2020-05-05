function y=car3(x,u)
dt=1/60;

% set hyperparameters
Ns=3; % states number
Nc=2; % control number
Np=30; % prediction number (horizion)
% dt=1/20; % sample period

Vx=x(1);
Vy=x(2);
delta=u(2);
dpsi=x(3);



% get matrix
m=1060;
Iz=1493.4;

lf=1.0462;
lr=1.2638;

a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;

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

w_wind=2.7e-4;
a_wind=w_wind*Vx.^2;
D1=-(Fyf*sin(delta))/m+dpsi*Vy-a_wind;
D2=(bf+br)/m;
D3=(lf*bf-lr*br)/Iz;



I=eye(3);

Ac=[0,0,0;
    0,A22,A23;
    0,A32,A33    ];

Bc=[1,0
       0,B26;
       0,B36  ];
   
 Dc=[D1,D2,D3]';
 
Ad=Ac*dt+I;
Bd=Bc*dt;
Dd=Dc*dt;

y=Ad*x+Bd*u+Dd;

end

