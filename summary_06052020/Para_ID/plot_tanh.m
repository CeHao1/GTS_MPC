function plot_tanh(x,y,b1)


range=min(x):0.0001:max(x);
result=b1(1)*tanh(b1(2)*range);
r2=b1(1)*b1(2)*range;
r2d=180/pi;
figure
hold on
plot(x*r2d,y,'.')
plot(range*r2d,result,'r','linewidth',2);
plot(range*r2d,r2,'k--','linewidth',2);
hold off
str=strcat('front/rear wheel lateral dynamic  :',num2str(b1),'   C\alpha :',num2str(b1(1)*b1(2)));
% title(str,'fontsize',15)
title('Tanh model','fontsize',15)
xlabel('Slip angle/ degree','fontsize',15);
ylabel('Lateral Force/ N','fontsize',15)
legend('Vehicle Lateral Force','Tanh Model','Linearized Model');

end