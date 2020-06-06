function [pos,edge,ds_ori]=initialize_centerline_demo(K)
% interval=1;
if nargin==0
    K=1000;
    plotflag=1;
    course_select=2;
else
    plotflag=0;
end

course_select=2;
if course_select==1

% 1
interval=0.001;
l1=200;
l2=150;
l3=l1-l2;

kap=0;
kap=[kap,ones(1,length(0:interval:50-interval))*0];

kap=[kap,(0:interval:l2-interval)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:l3-interval))*pi/(l1)];
kap=[kap,(l2-interval:-interval:0)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:100-interval))*0];

kap=[kap,(0:interval:l2-interval)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:l3-interval))*pi/(l1)];
kap=[kap,(l2-interval:-interval:0)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:50-0.5-interval))*0];


width=10;
wl=ones(1,length(kap))*width;
wr=ones(1,length(kap))*width;

elseif course_select==2
% 2
interval=0.001;
l1=200;
l2=150;
l3=l1-l2;

kap=0;
kap=[kap,ones(1,length(0:interval:100-interval))*0];

kap=[kap,(0:interval:l2-interval)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:l3-interval))*pi/(l1)];
kap=[kap,(l2-interval:-interval:0)*pi/(l1*l2)];
kap=[kap,ones(1,length(0:interval:50-interval))*0];

kap=[kap,ones(1,length(0:interval:50-interval))*0];
% 
l1_2=150;
l2_2=120;
l3_2=l1_2-l2_2;
kap=[kap,(0:-interval:-(l2_2-interval))*pi*0.5/(l1_2*l2_2)];
kap=[kap,-ones(1,length(0:interval:l3_2-interval))*0.5*pi/(l1_2)];
kap=[kap,(-(l2_2-interval):interval:0)*pi*0.5/(l1_2*l2_2)];
kap=[kap,ones(1,length(0:interval:50-interval))*0];
%

l1_0=150;
l2_0=90;
l3_0=l1_0-l2_0;
kap=[kap,(0:interval:l2_0-interval)*pi/(l1_0*l2_0)];
kap=[kap,ones(1,length(0:interval:l3_0-interval))*pi/(l1_0)];
kap=[kap,(l2_0-interval:-interval:0)*pi/(l1_0*l2_0)];
kap=[kap,ones(1,length(0:interval:20-interval))*0];
% 
% %
l1_3=200;
l2_3=150;
l3_3=l1_3-l2_3;
kap=[kap,ones(1,length(0:interval:140-2.473-interval))*0];
kap=[kap,(0:interval:(l2_3-interval))*pi*0.5/(l1_3*l2_3)];
kap=[kap,ones(1,length(0:interval:l3_3-interval))*0.5*pi/(l1_3)];
kap=[kap,((l2_3-interval):-interval:0)*pi*0.5/(l1_3*l2_3)];

kap=[kap,ones(1,length(0:interval:10+1.368-interval))*0];
kap=[kap,ones(1,length(0:interval:50-interval))*0];


width=20;
% width=5;
wl=ones(1,length(kap))*width;
wr=ones(1,length(kap))*width;
end
%%

s=0:interval:(length(kap)-1)*interval;



%% 
phi=0;
X=0;
Y=0;
Xl=X-wl(1)*sin(phi);
Yl=Y+wl(1)*cos(phi);
Xr=X+wr(1)*sin(phi);
Yr=Y-wr(1)*cos(phi);


for i=1:length(kap)-1
    phi(i+1)=phi(i)+interval*kap(i);
    X(i+1)=X(i)+interval*cos(phi(i));
    Y(i+1)=Y(i)+interval*sin(phi(i));
    Xl(i+1)=X(i+1)-wl(i+1)*sin(phi(i+1));
    Yl(i+1)=Y(i+1)+wl(i+1)*cos(phi(i+1));
    Xr(i+1)=X(i+1)+wr(i+1)*sin(phi(i+1));
    Yr(i+1)=Y(i+1)-wr(i+1)*cos(phi(i+1));
    
    
end
%% desample
ds_rate=round(length(kap)/K);
kap=kap(1:ds_rate:end);
phi=phi(1:ds_rate:end);
X=X(1:ds_rate:end);
Y=Y(1:ds_rate:end);
s=s(1:ds_rate:end);

Xl=Xl(1:ds_rate:end);
Yl=Yl(1:ds_rate:end);
Xr=Xr(1:ds_rate:end);
Yr=Yr(1:ds_rate:end);

wl=wl(1:ds_rate:end);
wr=wr(1:ds_rate:end);

% method 2

% kap=intp(kap,K);
% phi=intp(phi,K);
% X=intp(X,K);
% Y=intp(Y,K);
% s=intp(s,K);
% 
% Xl=intp(Xl,K);
% Yl=intp(Yl,K);
% Xr=intp(Xr,K);
% Yr=intp(Yr,K);
% 
% wl=intp(wl,K);
% wr=intp(wr,K);
interval_new=mean(diff(s));
%%
pos=[phi;X;Y;s;kap];
pos=[pos,pos(:,1)];
pos(1,end)=pos(1,1)+2*pi;
pos(4,end)=pos(4,end-1)+interval_new;

edge=[Xl;Yl;Xr;Yr;wl;wr];
edge=[edge,edge(:,1)];

% pos(6,1)=interval_new;
ds_ori=interval_new*ones(1,length(pos));
% ds_ori=extend(diff(pos(4,:)));
pos=[pos;ds_ori];

diff_psi=diff(pos(1,:));
diff_psi=[diff_psi,diff_psi(1)];
kap=diff_psi./ds_ori;

pos(5,:)=kap;

%%
if plotflag==1
    figure
    hold on
    
    plot(pos(2,:),pos(3,:),'b')
    plot(edge(1,:),edge(2,:),'r')
    plot(edge(3,:),edge(4,:),'g')
    plot(pos(2,1),pos(3,1),'ro')
   plot(pos(2,end),pos(3,end),'r*')
%     plot(X,Y,'b');
%     plot(Xl,Yl,'r');
%     plot(Xr,Yr,'g');
%     plot(X(1),Y(1),'ro');
%     plot(X(end),Y(end),'b*');

    axis equal
end

end