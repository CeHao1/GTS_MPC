clc
clear
close all

%% load data

h=get_human_ref();
% c=load('tokyo-1000-20');
% nc=load('Ugo-200tokyo-.mat');

c=load('tokyo-1000-50');
nc=load('Ugo-100tokyo-');

%% map

c=get_mapXY(c);
nc=get_mapXY(nc);
pos=c.pos;
edge=c.edge;

figure
hold on
h_cent=plot(pos(2,:),pos(3,:),'--k');
plot(edge(1,:),edge(2,:),'k');
plot(edge(3,:),edge(4,:),'k');
h_st=plot(pos(2,1),pos(3,1),'*r');
plot(pos(2,1),pos(3,1),'or');

h_h=plot(h.xout(4,:),h.xout(5,:),'color',[60,179,113]/255,'linewidth',1.5);

h_c=plot(c.CarX,c.CarY,'b','linewidth',1.5);

h_nc=plot(nc.CarX,nc.CarY,'r','linewidth',1.5);

hold off

axis equal

L1=legend([h_cent,h_h,h_c,h_nc,h_st],'Centerline','Human Best','Min \kappa','Min T','Start Point', ...
    'Location','Northwest');
set(L1,'fontsize',15);
xlabel('X / m ','fontsize',15);
ylabel('Y / m','fontsize',15);
title('Map of Tokyo expressway, clockwise is positive direction','fontsize',15);

%% Vxh_c

constraints=get_constraints(c.kap,c.Vx);
Vx_max=constraints(1,:);
s_pos=c.pos(4,:);


figure
hold on
V_cent=plot(s_pos,Vx_max,'k','linewidth',1.5);
V_h=plot(h.s,h.V,'Color',[60,179,113]/255,'linewidth',1.5);
V_c=plot(s_pos,c.xout(1,:),'b','linewidth',1.5);
V_nc=plot(s_pos,nc.xout(1,:),'r','linewidth',1.5);


hold off
axis([0,max(s_pos),35,60])

L2=legend([V_cent,V_h,V_c,V_nc],'Max Vx of Friction','Human Best','Min \kappa','Min T', ...
    'Location','Northeast');
set(L2,'fontsize',15);
xlabel('Course / m ','fontsize',15);
ylabel('Vx / m','fontsize',15);
title('Velocity Profile','fontsize',15);

%% kap

h.kap=get_kap(h.xout(4,:),h.xout(5,:));
c.kap=get_kap(c.CarX,c.CarY);
nc.kap=get_kap(nc.CarX,nc.CarY);

figure
hold on
K_cent=plot(s_pos,pos(5,:),'--k','linewidth',1);
K_h=plot(h.s,h.kap,'Color',[60,179,113]/255,'linewidth',1.5);
K_c=plot(s_pos,c.kap,'b','linewidth',1.5);
K_nc=plot(s_pos,nc.kap,'r','linewidth',1.5);

hold off

axis([0,4399,-0.011,0.011])


L3=legend([K_cent,K_h,K_c,K_nc],'Centerline','Human Best','Min \kappa','Min T', ...
    'Location','Northwest');
set(L3,'fontsize',15);
xlabel('Course / m ','fontsize',15);
ylabel('Curvature','fontsize',15);
title('Curvature','fontsize',15);



%% lap time

x_c=1:50;
x_nc=51:150;
x_h=1:150;

figure
hold on
T_h=plot(x_h,ones(150,1)*h.t,'Color',[60,179,113]/255,'linewidth',1.5);
T_c=plot(x_c,c.sum_out{5,1},'b','linewidth',1.5);
T_nc=plot(x_nc,nc.sum_out{5,1},'r','linewidth',1.5);
hold off

L4=legend([T_h,T_c,T_nc],'Human Best','Min \kappa','Min T', ...
    'Location','Northwest');
set(L4,'fontsize',15);
xlabel('Iterations','fontsize',15);
ylabel('Time / t','fontsize',15);
title('Lap Time','fontsize',15);

figure
hold on
plot(x_c,c.sum_out{5,1},'b','linewidth',1.5);
% plot(x_c,c.sum_out{5,1},'ko');
hold off
title('Min \kappa','Fontsize',12)


figure
hold on
plot(x_nc,nc.sum_out{5,1},'r','linewidth',1.5);
% plot(x_nc,nc.sum_out{5,1},'ko');
hold off
title('Min T','Fontsize',12)


%%  ey








