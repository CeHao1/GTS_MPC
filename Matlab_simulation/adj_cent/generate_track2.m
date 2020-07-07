function [para,pc]=generate_track2()

interval=0.01;
x1=(0:interval:5)';
y1=zeros(length(x1),1)+2;
theta=(pi/2:-0.001:-pi/2)';
theta(1)=[];
% theta(end)=[];
x2=2*cos(theta)+5;
y2=2*sin(theta);
x3=(5:-interval:0)';
y3=zeros(length(x1),1)-2;

pc.X=[x1;x2;x3];
pc.Y=[y1;y2;y3];
% Ns=60;
% pc.X=smooth(pc.X,Ns);
% pc.Y=smooth(pc.Y,Ns);
para=get_para(pc);




end