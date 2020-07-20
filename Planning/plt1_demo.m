close all

Np=length(pos);
psi=pos(1,:);
s_pos=pos(4,:);

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
        plot(((xout(i,:)))/pi*180)
    
    elseif i==4
        hold on
        plot(((xout(i,:)))/pi*180,'b')
%         plot(  (xout(7,:)-psi)/pi*180,'r'  );
        hold off
        
     elseif i==7
        hold on
        plot((psi)/pi*180,'b')
%         plot(((xout(i,:)))/pi*180,'r')
        plot((psi+xout(4,:))/pi*180,'r')
    elseif i==1
        
        hold on
%         plot(constraints(1,:),'b')
        plot(xout(i,:),'r')
        hold off   
        axis([1,length(xout),min(xout(i,:))-10,max(xout(i,:))+10])
    else
        plot(xout(i,:))
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
plot(edge(1,:),edge(2,:),'r');
plot(edge(3,:),edge(4,:),'m');
% plot(pos(2,1),pos(3,1),'ro');
h2=plot(CarX,CarY,'k','linewidth',2);
h3=plot(CarX(1),CarY(1),'ro');
plot(CarX(end),CarY(end),'r*');
hold off
axis equal
plotgap=50;
axis([min(pos(2,:))-plotgap,max(pos(2,:))+plotgap,min(pos(3,:))-plotgap,max(pos(3,:))+plotgap])
xlabel('X (m)','fontsize',20)
ylabel('Y (m)','fontsize',20)
title('Track , anti-cloclwise','fontsize',20)
legend([h1,h2,h3],'Centerline','Planned Path','Start Point')
%% get new kap

% kap2=xout(3,:)./xout(1,:);

dx=extend(diff(CarX));
dy=extend(diff(CarY));
d2x=extend(diff(dx));
d2y=extend(diff(dy));
kap3=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;

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
h2=plot(s_pos,kap0,'k','linewidth',2);
% h3=plot(s_pos,add_kap,'b','linewidth',2);
h5=plot(s_pos,kap3,'b','linewidth',2);
h4=plot(s_pos,kap0+add_kap,'m','linewidth',2);

hold off
% legend('centerline ','planned')
legend([h2,h4,h5],'d\psi/V','d\psi/V+\Delta \beta/\Delta s','kap from waypoints');
% legend('centerline ','\Delta \psi/\Delta s','d\psi /Vx','\Delta \psi/\Delta s new')
title('Curvature','fontsize',20)
xlabel('Course s (m)','fontsize',20)
ylabel('\kappa','fontsize',20)

%%
figure
hold on
plot(ds_ori,'b')
plot(ds_new,'r')
hold off

legend('original ds','new ds')

%%
% sum_dt=ds_ori./xout(1,1:end-1);
% sum(sum_dt)





