close all
clear
addpath('./fnc')




load('tokyo-2000-30_93')
[J1,beta1,an1]=get_cost(sum_out);
kap1=kap;

load('tokyo-2000-30_97')
[J2,beta2,an2]=get_cost(sum_out);
kap2=kap;

figure
hold on
h1=plot(J1,'b','linewidth',2);
h2=plot(J2,'r','linewidth',2);
hold off


titlename=strcat('V');

legend([h1,h2],'With Vy','No Vy')
title(titlename,'fontsize',20)
xlabel('2000 waypoints','fontsize',20)

costJ1=sum(J1);
costJ2=sum(J2);
% [costJ1,costJ2]

%%
figure
hold on
h1=plot(kap1,'b','linewidth',2);
h2=plot(kap2,'r','linewidth',2);
hold off


titlename=strcat('Curvature');

legend([h1,h2],'With Vy','No Vy')
title(titlename,'fontsize',20)
xlabel('2000 waypoints','fontsize',20)


% [sum((kap1).^2),sum((kap2).^2)]

%%

figure
hold on
h1=plot(an1,'b','linewidth',2);
h2=plot(an2,'r','linewidth',2);
hold off


titlename=strcat('Vy*dpsi');

legend([h1,h2],'With Vy','No Vy')
title(titlename,'fontsize',20)
xlabel('2000 waypoints','fontsize',20)


