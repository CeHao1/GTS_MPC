close all
% load('name')


addpath('./fnc');

%% map
figure
hold on
plot(pos(2,:),pos(3,:),'b--');
plot(edge(1,:),edge(2,:),'m');
plot(edge(3,:),edge(4,:),'b');

for iter=1:iter_all
    [CarX,CarY]=get_mapXY(pos,sum_out{1,iter});
    if iter==iter_all
         plot(CarX,CarY,'r','linewidth',2)
         plot(CarX(1),CarY(1),'bo','linewidth',2)
    else
        plot(CarX,CarY,'k')
    end
    
end

hold off
axis equal


%% kap
figure
hold on
plot(pos(5,:),'b','linewidth',1.5)
for iter=1:iter_all
    plot(sum_out{3,iter},'r')
end
hold off
title('curvature','fontsize',15)
xlabel('course/m','fontsize',15)
ylabel('\kappa','fontsize',15)


%% time
t_u=[29.4 ,33.1 ,29.4 ,33.1 ,22.3 ,20.1 ,19.0 ,17.9 ,16.9 ,16.1 ,15.4 ,14.9 ,14.2 ,13.7 ,13.2 ,12.8 ,12.4 ,11.9 ,11.5 ,11.1 ,10.8 ,10.6 ,10.3 ,10.1 ,10.0 ,9.8 ,9.5 ,9.4 ,9.1 ,9.1 ,9.0 ,8.8 ,8.8 ,8.3 ,8.2 ,7.9 ,7.8 ,7.8 ,7.8 ,7.6 ,7.3 ,7.4 ,7.4 ,7.5 ,7.3 ,7.3 ,7.4 ,7.1 ,7.1 ,7.2 ,7.0 ,7.0 ,7.0 ,7.0 ];

time=sum_out{5,1};
figure
hold on
h1=plot(time,'b','linewidth',2);
% plot(time,'ro','linewidth',2)
h2=plot(t_u,'k','linewidth',2);
% plot(t_u,'mo','linewidth',2)
hold off

title(strcat('lap time: ',num2str(sum_out{5,1}(end))),'fontsize',20)
% title('Lap time','fontsize',20)
xlabel('Iterations','fontsize',20)
ylabel('Time / s','fontsize',20)
legend([h1,h2],'\fontsize{15}IPC','\fontsize{15}LMPC')

