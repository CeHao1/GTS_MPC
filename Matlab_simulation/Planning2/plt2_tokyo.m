close all

addpath('./fnc')
Np=length(pos);
psi=pos(1,:);
s_pos=pos(4,:);
addpath('./csv');
[xout_ref,uout_ref]=human_ref();
% xout_ref=[Vx';Vy';av2';X';Y';s',kap];
% uout_ref=[thr';brk';steer'];

if ~exist('xout')
    xout=sum_out{1,end};
    uout=sum_out{2,end};
end


%%
name={'Vx','Vy','dpsi','epsi','ey','steer','psi'};
figure
for i=1:7
subplot(2,4,i)
    if  i==6 
        plot(s_pos,((xout(i,:)))/pi*180)
    
    elseif i==4
        hold on
        plot(s_pos,((xout(i,:)))/pi*180,'b')

        hold off
        
     elseif i==7
        hold on
        plot(s_pos,(psi)/pi*180,'b')

        plot(s_pos,(psi+xout(4,:))/pi*180,'r')

        legend('centerline','simulation')
        
    elseif i==1 %Vx
        hold on
        plot(xout_ref(6,:),xout_ref(1,:),'b')
        plot(s_pos,xout(i,:),'r')
        hold off   
        bias=10;
%         axis([1,length(xout),min(xout(i,:))-bias,max(xout(i,:))+bias])
%         legend('Human','Planned Path',2)
    else
        plot(s_pos,xout(i,:))
    end

title(name{i},'fontsize',20)
end
% subplot(247)
% plot(uout(2,:))
% title('dsteer')

subplot(248)
hold on
plot(uout(1,:),'b')
% plot(diff(xout(1,:)),'r')
hold off
title('d steer','fontsize',20)

%%

% Car_psi=xout(7,:);
Car_ey=xout(5,:);
% CarX=pos(2,:)-sin(Car_psi).*Car_ey;
% CarY=pos(3,:)+cos(Car_psi).*Car_ey;

X=pos(2,:);
Y=pos(3,:);

CarX=X-sin(psi).*Car_ey;
CarY=Y+cos(psi).*Car_ey;

figure
hold on
h1=plot(pos(2,:),pos(3,:),'b--');
plot(edge(1,:),edge(2,:),'m');
plot(edge(3,:),edge(4,:),'g');
h2=plot(xout_ref(4,:),xout_ref(5,:),'r','linewidth',1.5);
h3=plot(CarX,CarY,'k','linewidth',1.5);

plot(CarX(1),CarY(1),'ro');
plot(CarX(end),CarY(end),'r*');
hold off
legend([h1,h2,h3],'Centerline','Human','Planned Path')

axis equal
plotgap=50;
axis([min(pos(2,:))-plotgap,max(pos(2,:))+plotgap,min(pos(3,:))-plotgap,max(pos(3,:))+plotgap])
xlabel('X (m)','fontsize',20)
ylabel('Y (m)','fontsize',20)
title('Track , Cloclwise','fontsize',20)


%% get new kap

dx=extend(diff(CarX));
dy=extend(diff(CarY));
d2x=extend(diff(dx));
d2y=extend(diff(dy));
kap3=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;
kap3=smooth(kap3,50);

if ~exist('ds_new')
ds_new=extend(sqrt(diff(CarX).^2+diff(CarY).^2));
end

Vx=xout(1,:);
Vy=xout(2,:);
dpsi=xout(3,:);
kap0=dpsi./Vx;
beta=asin(Vy./Vx);
add_kap=extend(diff(beta))./ds_new;

figure
hold on
plot(s_pos,pos(5,:),'b')
plot(xout_ref(6,:),xout_ref(7,:)*1.03,'r')
plot(s_pos,kap,'k','linewidth',2)
% plot(s_pos,add_kap,'b','linewidth',2)
% plot(s_pos,kap0+add_kap,'m','linewidth',2)
% plot(s_pos,kap3,'k','linewidth',2)


hold off

legend('Centerline ','Human','Planned Path')
% title('Curvature','fontsize',20)
xlabel('Course/ m','fontsize',15)
ylabel('Curvature ','fontsize',15)

%%
% figure
% hold on
% plot(ds_ori,'b')
% plot(ds_new,'r')
% hold off
% 
% legend('original ds','new ds')

%%
% sum_dt=ds_ori./xout(1,1:end-1);
% sum(sum_dt)

constraints=get_constraints(kap,Vx);
Vx_max=constraints(1,:);

figure
 hold on
 plot(s_pos,Vx_max,'k','linewidth',1.5)
plot(xout_ref(6,:),xout_ref(1,:),'b','linewidth',2)
plot(s_pos,xout(1,:),'r','linewidth',2)
hold off   
axis([0,max(s_pos),35,60])

% title('Vx','fontsize',20)
xlabel('Course / m','fontsize',20)
ylabel('Velocity','fontsize',20)
legend('Velocity limit','Human best','Planned velocity profile')




