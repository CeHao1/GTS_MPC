close all
% load('name')


addpath('./fnc');
addpath('./csv');
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
time=sum_out{5,1};
figure
hold on
plot(smooth(time,1),'b','linewidth',2)
plot(smooth(time,1),'ro','linewidth',2)
hold off

% title(strcat('lap time: ',num2str(sum_out{5,1}(end))),'fontsize',15)
title('Lap time','fontsize',20)
xlabel('Iterations','fontsize',20)
ylabel('Time / s','fontsize',20)
