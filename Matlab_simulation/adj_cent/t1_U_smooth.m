clc;clear 
close all

interval=0.1;
x1=(0:interval:5)';
y1=zeros(length(x1),1)+2;
theta=(pi/2:-0.05:-pi/2)';
x2=2*cos(theta)+5;
y2=2*sin(theta);
x3=(5:-interval:0)';
y3=zeros(length(x1),1)-2;

pc.X=[x1;x2;x3];
pc.Y=[y1;y2;y3];


Ns=60;
X=smooth(pc.X,Ns);
Y=smooth(pc.Y,Ns);

figure
hold on

plot(pc.X,pc.Y,'r--')
plot(X,Y,'k')
hold off

axis equal