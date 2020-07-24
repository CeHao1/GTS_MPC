close all
clear 
clc

%% load data
d_tr=load('Track_without_kap');
d_u=load('Track_Ugo.mat');
d_c=load('Ugo-40-cvx1.mat');
d_T=load('Ugo-40minT.mat');
d_V=load('Ugo-40maxVx.mat');

pos=d_c.pos;
edge=d_c.edge;

%% map
d_c=get_mapXY(d_c);
d_T=get_mapXY(d_T);
d_V=get_mapXY(d_V);

figure
hold on
h_cent=plot(pos(2,:),pos(3,:),'--','color',[66,99,33]/255);
plot(edge(1,:),edge(2,:),'color',[66,99,33]/255);
plot(edge(3,:),edge(4,:),'color',[66,99,33]/255);
st=plot(pos(2,1),pos(3,1),'r*');
plot(pos(2,1),pos(3,1),'ro')

h_u=plot(d_u.X_u,d_u.Y_u,'m--','linewidth',2);
h_c=plot(d_c.CarX,d_c.CarY,'k','linewidth',2);
h_T=plot(d_T.CarX,d_T.CarY,'r','linewidth',2);
h_V=plot(d_V.CarX,d_V.CarY,'b','linewidth',2);

hold on
axis equal
Hl=legend([h_cent,h_u,h_c,h_T,h_V,st],'Centerline','LMPC','Min curv','Min T','Max Vx','Start Point');
set(Hl,'fontsize',15)
title('Demo Track, anti-clockwise is positive direction','fontsize',15)
xlabel('X / m','fontsize',15)
ylabel('Y / m','fontsize',15)
%% Vx
d_T.V=sqrt(d_T.xout(1,:).^2+d_T.xout(2,:).^2);
d_V.V=sqrt(d_V.xout(1,:).^2+d_V.xout(2,:).^2);

s_pos=d_c.pos(4,:);
figure
hold on
v_u=plot(d_u.s_u,d_u.Vx_u,'m','linewidth',1.5);
v_c=plot(s_pos,d_c.V,'k','linewidth',1.5);
v_T=plot(s_pos,d_T.V,'r','linewidth',1.5);
v_V=plot(s_pos,d_V.V,'b','linewidth',1.5);

hold off

v_l=legend([v_u,v_c,v_T,v_V],'LMPC','Min Curv','Min T','Max V');
set(v_l,'fontsize',15);
title('Velocity Profile','fontsize',15)
xlabel('Course / m','fontsize',15)
ylabel('Velcotiy / ms^-^1','fontsize',15)

%% kap

d_u.kap= get_kap(d_u.X_u,d_u.Y_u);

figure
hold on
k_cent=plot(s_pos,pos(5,:),'linewidth',1.5,'color',[66,99,33]/255);
k_u=plot(d_u.s_u,d_u.kap,'m','linewidth',1.5);
k_c=plot(s_pos,d_c.kap,'k','linewidth',1.5);
k_T=plot(s_pos,d_T.kap,'r','linewidth',1.5);
k_V=plot(s_pos,d_V.kap,'b','linewidth',1.5);
hold off

kl=legend([k_cent,k_u,k_c,k_T,k_V],'Centerline','LMPC','Min curv','Min T','Max Vx');
set(kl,'fontsize',15)
title('Curvature','fontsize',15)
xlabel('Course / m','fontsize',15)
ylabel('Curvature','fontsize',15)


%% lap time

d_u.t_u=[29.4 ,33.1 ,29.4 ,33.1 ,22.3 ,20.1 ,19.0 ,17.9 ,16.9 ,16.1 ,15.4 ,14.9 ,14.2 , ...
    13.7 ,13.2 ,12.8 ,12.4 ,11.9 ,11.5 ,11.1 ,10.8 ,10.6 ,10.3 ,10.1 ,10.0 ,9.8 ,9.5 ,9.4 ,...
    9.1 ,9.1 ,9.0 ,8.8 ,8.8 ,8.3 ,8.2 ,7.9 ,7.8 ,7.8 ,7.8 ,7.6 ,7.3 ,7.4 ,7.4 ,7.5 ,7.3 ,7.3 ,7.4 ,7.1 , ...
    7.1 ,7.2 ,7.0 ,7.0 ,7.0 ,7.0 ];
d_c.time=d_c.sum_out{5,1};
d_T.time=d_T.sum_out{5,1};
d_V.time=d_V.sum_out{5,1};

figure
hold on
tm_u=plot(d_u.t_u,'m','linewidth',1.5);
tm_c=plot(d_c.time,'k','linewidth',1.5);
tm_T=plot(d_T.time,'r','linewidth',1.5);
tm_V=plot(d_V.time,'b','linewidth',1.5);
hold off

tl=legend([tm_u,tm_c,tm_T,tm_V],'LMPC','Min Curv','Min T','Max V');
set(tl,'fontsize',15);

title('Lap Time at each Iteration','fontsize',15)
xlabel('Iterations','fontsize',15);
ylabel('Time / s','fontsize',15);




