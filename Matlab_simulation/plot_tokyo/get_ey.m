clc
clear 
close all

%%
h=get_human_ref();
% c=load('tokyo-1000-20');
% nc=load('Ugo-200tokyo-.mat');

c=load('tokyo-1000-50');
nc=load('Ugo-100tokyo-');
c=get_mapXY(c);
nc=get_mapXY(nc);

s_pos=c.pos(4,:);
%%

name2p='centerline2_tokyo.csv';
map_data=csvread(name2p);

cX=map_data(:,1);
cY=map_data(:,2);
ref=[cX,cY];

ref1=[ref;ref(1:10,:)];
ref2=[ref(end-10:end,:);ref];


%% 
% human

for i=1:length(h.X)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_h(i),~]=find_near_point_0([h.X(i),h.Y(i)],ref);  
end

% min kap

for i=1:length(c.CarX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_c(i),~]=find_near_point_0([c.CarX(i),c.CarY(i)],ref);  
end

% min T
for i=1:length(nc.CarX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_nc(i),~]=find_near_point_0([nc.CarX(i),nc.CarY(i)],ref);  
end



figure
hold on
e1=plot(h.s,ones(1,length(h.s))*3.75,'k--','linewidth',2);
plot(h.s,-ones(1,length(h.s))*3.75,'k--','linewidth',2);
e2=plot(h.s,ones(1,length(h.s))*3.25,'--','linewidth',2,'color',[138,43,226]/255);
plot(h.s,-ones(1,length(h.s))*3.25,'--','linewidth',2,'color',[138,43,226]/255);

e_h=plot(h.s,ey_h,'Color',[60,179,113]/255,'linewidth',1.5);
e_c=plot(s_pos(1:end-1),ey_c(1:end-1),'b','linewidth',1.5);
e_nc=plot(s_pos(1:end-1),ey_nc(1:end-1),'r','linewidth',1.5);

hold off

L4=legend([e1,e2,e_h,e_c,e_nc],'Edge of Track','Edge with Safe Gap','Human Best',...
    'Min \kappa','Min T');
set(L4,'fontsize',15);
xlabel('Course / m ','fontsize',15);
ylabel('Distance / m','fontsize',15);
title('Distance to centerline','fontsize',15);



