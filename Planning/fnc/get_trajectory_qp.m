function [xout,uout]=get_trajectory_qp(pos,edge,x_last,ds_ori,constraints)

Ns=7; % states number
Nc=2; % control number
Np=length(pos)-1; % prediction number (horizion)
% pos=[phi;X;Y;s;kap];

edge_l=-edge(5,:);
edge_r=edge(6,:);

%%

% get matrix
m=1060;
Iz=1493.4;

% L=2.3100;
lf=1.0462;
lr=1.2638;

a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;

A_combined1=repmat({zeros(Ns,Ns)},Np+1,Np+1);
A_combined2=repmat({zeros(Ns,Nc)},Np+1,Np+1);
I=eye(Ns);

dt=x_last(1,1)/ds_ori(1);
B0=sparse([6],[2],[1],Ns,Nc)*dt;
B0=full(B0);

A_combined1{1,1}=I;
A_combined2{1,1}=-B0;

% states

D_combined=zeros((Np+1)*Ns,1);

for i=2:Np+1
    
    Vx=x_last(1,i);
    Vy=x_last(2,i);
    dpsi=x_last(3,i);
    epsi=x_last(4,i);
    ey=x_last(5,i);
    delta=x_last(6,i);
    
    ds_now=ds_ori(i);
    dt=ds_now/Vx;
    kap=pos(5,i)*pos(6,1)/ds_now;
    
    
    xk=x_last(:,i);
    %% 
    
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
%     R41=-cos(epsi)*kap/den;
%     R42=sin(epsi)*kap/den;
%     R44=(Vx*sin(epsi)+Vy*cos(epsi))*kap/den;
%     R45=(Vx*cos(epsi)-Vy*sin(epsi))*kap^2/den^2;
    R51=sin(epsi);
    R52=cos(epsi);
    R54=Vx*cos(epsi)-Vy*sin(epsi);

%     partial_epsi=[R41,R42,1,R44,R45,0,0];
    partial_ey=[R51,R52,0,R54,0,0,0];

%     depsi=dpsi-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
    dey=Vx*sin(epsi)+Vy*cos(epsi);

%     D4=depsi-partial_epsi*x0;
%     D5=dey-partial_ey*x0;

    D4=-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
    D5=dey-partial_ey*xk+R51*Vx;

    
    Ac=[[0,0,0,0,0,0,0];
        [0,A22,A23,0,0,B26,0];
        [0,A32,A33,0,0,B36,0];
%         partial_epsi;
%         partial_ey;
        [0,0,1,0,0,0,0];
        [0,R52,0,R54,0,0,0];
        
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

    A_combined1{i,i}=I;
    A_combined1{i,i-1}=-Ad;
    A_combined2{i,i}=-Bd;
    
    D_combined{i,1}=Dd;
    
    
end
A_combined=[A_combined1,A_combined2];
A_qp=cell2mat(A_combined);
D_qp=cell2mat(D_combined);

% equality of end and start
% x0=x(Np)
I_last=zeros(Ns,(Np+1)*(Ns+Nc));  
I_last(1:Ns,1:Ns)=eye(Ns);
I_last(1+Np*Ns:(Np+1)*Ns,1+Np*Ns:(Np+1)*Ns)=-eye(Ns);
% psi - epsi=psi_ref
psi_last=sparse([1,1],[7,4],[1,-1],1,(Np+1)*(Ns+Nc));

A_qp=[A_qp;I_last;psi_last];
D_qp=[D_qp;0;pos(1,1)];

% boundary
ub_qp=ones((Np+1)*(Ns+Nc),1)*inf;
lb_qp=-ones((Np+1)*(Ns+Nc),1)*inf;

% Vx
ub_qp(1:Ns:(Np+1)*Ns,1)=constraints(1,:)';
lb_qp(1:Ns:(Np+1)*Ns,1)=zeros(1,1+Np)';

% ax
ub_qp((Np+1)*Ns+1:Nc:(Np+1)*(Ns+Nc),1)=constraints(2,:)';
lb_qp((Np+1)*Ns+1:Nc:(Np+1)*(Ns+Nc),1)=constraints(3,:)';

%steer
ub_qp(6:Ns:(Np+1)*Ns,1)=ones(Np+1,1)*6/180*pi;
lb_qp(6:Ns:(Np+1)*Ns,1)=-ones(Np+1,1)*6/180*pi;

%dsteer
ub_qp((Np+1)*Ns+2:Nc:(Np+1)*(Ns+Nc),1)=ones(Np+1,1)*10/180*pi;
lb_qp((Np+1)*Ns+2:Nc:(Np+1)*(Ns+Nc),1)=-ones(Np+1,1)*10/180*pi;

% ey
ub_qp(5:Ns:(Np+1)*Ns,1)=edge_l';
lb_qp(5:Ns:(Np+1)*Ns,1)=edge_r';


% cost function
%H : 
w_kap=1.2e1;
w_Vx=1e-5;
w_delta=1e0;

H=zeros((Np+1)*(Nc+Ns),Np*2+1);
% kap
for i=1:Np
    ds_now=ds_ori(i);
    H(i,7+(i-1)*Ns)=-1*w_kap/ds_now;
    H(i,7+(i)*Ns)=1*w_kap/ds_now;
end
% d steer
i=1:Np+1;
H(i+Np,(Np+1)*Ns+2+(i-1)*Nc)=w_delta;

% Vx
f=zeros(1,(Np+1)*(Ns+Nc));
f(1,1:Ns:(Np+1)*Ns)=-w_Vx;


% options = optimoptions('fmincon');
options = optimset('Algorithm','active-set','display','off'); 

xout=quadprog(H,f,[],[],A_qp,D_qp,lb_qp,ub_qp,[],options);


end