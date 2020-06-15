function [xout,uout]=lateral(pos,edge,x_last,ds_ori)

Ns=5; % states number
Nc=1; % control number
Np=length(pos); % prediction number (horizion)
% pos=[phi;X;Y;s;kap];

gap=0.05;
edge_l=edge(5,:)-gap;
edge_r=-edge(6,:)+gap;

dsteer_limit=50;
steerlimit=50;
%%

m  = 1.98;
lf = 0.125;
lr = 0.125;
Iz = 0.024;
a11=7.3267;
a12=1.3253;
a21=7.3267;
a22=1.3253;

A_combined1=repmat({zeros(Ns,Ns)},Np,Np);
A_combined2=repmat({zeros(Ns,Nc)},Np,Np-1);
I=eye(Ns);

init_I=diag([1,1,1,1,1]);
A_combined1{1,1}=init_I;
A_combined1{1,Np}=-init_I;

% D_combined=zeros((Np)*Ns,1);
D_combined=repmat({zeros(Ns,1)},Np,1);

for i=2:Np
    index=i-1;
    
    Vx=x_last(1,index);
    Vy=x_last(2,index);
    V=sqrt(Vx^2+Vy^2);
    dpsi=x_last(3,index);
    epsi=x_last(4,index);
%     ey=x_last(5,i);
    delta=x_last(6,index);
    
    ds_now=ds_ori(index);
%     dt=ds_now/Vx;
    dt=ds_now/V;
    kap=pos(5,index)*pos(6,index)/ds_now; % relavtive curvature of centerline
    
    
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
%     D1=-(Fyf*sin(delta))/m+dpsi*Vy;
    D2=(bf+br)/m;
    D3=(lf*bf-lr*br)/Iz;

%     den=1-kap*ey;
%     R41=-cos(epsi)*kap/den;
%     R42=sin(epsi)*kap/den;
%     R44=(Vx*sin(epsi)+Vy*cos(epsi))*kap/den;
%     R45=(Vx*cos(epsi)-Vy*sin(epsi))*kap^2/den^2;
    R51=sin(epsi);
    R52=cos(epsi);
    R54=Vx*cos(epsi)-Vy*sin(epsi);

%     partial_epsi=[R41,R42,1,R44,R45,0,0];
    partial_ey=[R51,R52,0,R54,0,0];

%     depsi=dpsi-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
    dey=Vx*sin(epsi)+Vy*cos(epsi);

%     D4=depsi-partial_epsi*x0;
%     D5=dey-partial_ey*x0;

%     D4=-(Vx*cos(epsi)-Vy*sin(epsi))*kap/den;
    D4=-V*kap;
    D5=dey-partial_ey*xk+R51*Vx;
%     D5=0;

    
    Ac=[ [A22,A23,0,0,B26];
        [A32,A33,0,0,B36];
%         partial_epsi;
%         partial_ey;
        [0,1,0,0,0];
        [R52,0,R54,0,0];
%         [cos(epsi),0,Vx,0,0];
        [0,0,0,0,0]];
    
    Bc=[0,0,0,0,1]';
    Dc=[D2,D3,D4,D5,0,]';
        
    Ad=Ac*dt+eye(Ns);
    Bd=Bc*dt;
    Dd=Dc*dt;

    A_combined1{i,i}=I;
    A_combined1{i,i-1}=-Ad;
    A_combined2{i,i-1}=-Bd;
    
    D_combined{i,1}=Dd;

end
A_combined=[A_combined1,A_combined2];
A_qp=cell2mat(A_combined);
D_qp=cell2mat(D_combined);

%
ub_qp=ones(Np*Ns+(Np-1)*Nc,1)*100;
lb_qp=-ones(Np*Ns+(Np-1)*Nc,1)*100;

%
%steer
ub_qp(5:Ns:(Np)*Ns,1)=ones(Np,1)*steerlimit/180*pi;
lb_qp(5:Ns:(Np)*Ns,1)=-ones(Np,1)*steerlimit/180*pi;

% epsi
ub_qp(3:Ns:(Np)*Ns,1)=ones(Np,1)*90/180*pi;
lb_qp(3:Ns:(Np)*Ns,1)=-ones(Np,1)*90/180*pi;

%dsteer
ub_qp((Np)*Ns+1:Nc:Np*Ns+(Np-1)*Nc,1)=ones(Np-1,1)*dsteer_limit/180*pi;
lb_qp((Np)*Ns+1:Nc:Np*Ns+(Np-1)*Nc,1)=-ones(Np-1,1)*dsteer_limit/180*pi;

% ey
ub_qp(4:Ns:(Np)*Ns,1)=edge_l';
lb_qp(4:Ns:(Np)*Ns,1)=edge_r';

% cost
w_kap=1;

w_delta=1e1;
% w_Vy=0.015;

% w_delta=0;
w_Vy=0;
w_steer=1e1;

weight=zeros(1,Np*Ns+(Np-1)*Nc);
% weight(1,(2:Ns:Np*Ns))=w_kap./x_last(1,:);
weight(1,(2:Ns:Np*Ns))=w_kap./x_last(1,:).^2;   
weight(1,(5:Ns:Np*Ns))=w_steer;   
weight(1,(Np*Ns+1:Nc:Np*Ns+(Np-1)*Nc))=w_delta;

% Vy
weight(1,(1:Ns:Np*Ns))=w_Vy;

H=diag(weight);

%%

% options = optimset('Algorithm','active-set','display','off'); 
% options = optimset('Algorithm','active-set'); 
% xout=quadprog(H,[],[],[],A_qp,D_qp,lb_qp,ub_qp,[],options);




%%
% cvx_begin 
cvx_begin quiet
% cvx_begin
    variable x(Np*Ns+(Np-1)*Nc,1)
    J=x'*H*x;
%     J=norm(sqrt(weight').*x)
    minimize(J)
    subject to
        A_qp*x==D_qp
        lb_qp<=x<=ub_qp
cvx_end


%%
xout(1,:)=x_last(1,:);
for i=1:Ns
    xout(i+1,:)=x(i:Ns:Np*Ns);
end

for i=1:Nc
    uout(i,:)=x(Np*Ns+i:Nc:Np*Ns+(Np-1)*Nc);
end



end