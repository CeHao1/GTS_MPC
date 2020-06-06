

data0=csvread('reference1.csv');
map_data=csvread('centerline1.csv');
reference=data0;

figure
% subplot(2,3,[1:2,4:5])
hold on
h1=plot(reference(:,2),reference(:,3),'r','linewidth',2);
% h2=plot(X,Y,'k','linewidth',2);
plot(map_data(:,1),map_data(:,2),'b--');
plot(map_data(:,3),map_data(:,4),'r--');
plot(map_data(:,5),map_data(:,6),'m--');

hold off