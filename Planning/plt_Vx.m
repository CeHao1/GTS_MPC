close all

[xout_ref,uout_ref]=human_ref();

figure
hold on
plot(xout_ref(6,:),xout_ref(1,:),'b')

load('tokyo-1000-15-a130')
xout=sum_out{1,end};
s_pos=pos(4,:);
plot(s_pos,xout(1,:),'r')


load('tokyo-1000-15-a135')
xout=sum_out{1,end};
s_pos=pos(4,:);
plot(s_pos,xout(1,:),'k')

load('tokyo-4000-15-a135')
xout=sum_out{1,end};
s_pos=pos(4,:);
plot(s_pos,xout(1,:),'m')


hold off   
% bias=10;

