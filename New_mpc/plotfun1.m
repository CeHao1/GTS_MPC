r2d=180/pi;

close all
addpath('./map')
name2p='centerline2_tokyo.csv';
map_data=csvread(name2p);


% reference=csvread('reference1.csv');
reference=csvread('reference2000_93.csv');

%%
dX=extend(diff(X));
dY=extend(diff(Y));
psi2=atan2(dY,dX);


referenceVx=spline(reference(:,1),reference(:,4),course_v);
referencepsi=spline(reference(:,1),reference(:,6),course_v);
epsi_ref=wrapToPi(psi-referencepsi);


%%

figure
% subplot(2,3,[1:2,4:5])
hold on
h1=plot(reference(:,2),reference(:,3),'r','linewidth',2);
h2=plot(X,Y,'k','linewidth',2);
plot(X(1),Y(1),'k*','linewidth',2);

plot(map_data(:,1),map_data(:,2),'b--');
plot(map_data(:,3),map_data(:,4),'r--');
plot(map_data(:,5),map_data(:,6),'m--');

hold off
axis equal
plotdist=100;
axis([min(map_data(:,1))-plotdist,max(map_data(:,1))+plotdist,min(map_data(:,2))-plotdist,max(map_data(:,2))+plotdist]);
title('X-Y position','fontsize',15)
legend([h1,h2],'reference','real path')


%%

pltflag1=1;

if pltflag1==1

% ey_h(1:50)=smooth(ey_h(1:50),40);
% ey_c(1:50)=smooth(ey_c(1:50),40);


cX=map_data(:,1);
cY=map_data(:,2);
ref=[cX,cY];


vX=X;
vY=Y;
s=course_v;

ref1=ref;
ref2=[ref(end-10:end,:);ref];
ey_c=[];
for i=1:length(vX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_c(i),~]=find_near_point_0([vX(i),vY(i)],ref);  
end


rX=reference(:,2);
rY=reference(:,3);
rs=reference(:,1);
ey_r=[];
for i=1:length(rX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_r(i),~]=find_near_point_0([rX(i),rY(i)],ref);  
end

figure
hold on
h1=plot(s,ones(1,length(s))*3.75,'k-','linewidth',2);
plot(s,-ones(1,length(s))*3.75,'k-','linewidth',2);
h2=plot(s,ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255);
plot(s,-ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255)

% h3=plot(hs,ey_h,'--','linewidth',2,'Color',[60,179,113]/255);
h4=plot(rs,ey_r,'b-','linewidth',2);
% h5=plot(s,ey_c,'-','linewidth',2,'color',[255,69,0]/255);
h5=plot(s,ey_c,'-r','linewidth',2);

hold off
% title('ey of centerline','fontsize',15)
% legend([h1,h2,h3,h4,h5],'\fontsize {15} Width of Track','\fontsize {15} Width with Safe Gap','\fontsize {15} Human','\fontsize {15} Treajectory Planning','\fontsize {15} MPC')
xlabel('Course / m','fontsize',20)
ylabel('Lateral path deviation / m','fontsize',20)

end


%% ey to reference

ref_r=reference(:,2:3);
ref_r=[ref_r,reference(:,1)];
vX=X;
vY=Y;

for i=1:length(vX)
    [ey_2r(i),s_ref(i)]=find_near_point([vX(i),vY(i)],ref_r);  
end

figure
hold on
plot(ey_2r,'b')
plot(ey,'r')
hold off
title('ey')
legend('ture','api')

%%
figure
subplot(241)
hold on
plot(referenceVx,'k--','linewidth',2)
plot(Vx,'b')
plot(Vx2,'r')

hold off
title('Vx')

subplot(242)
hold on
plot(Vy)
plot(Vy2,'r')
hold off
title('Vy')

subplot(243)
hold on
plot(psi*r2d,'b')
plot(psi2*r2d,'r')
hold off

title('psi')


subplot(244)
hold on
plot(acc,'b')
plot(thr-brk,'r')
hold off
title('acc')


subplot(245)
hold on
plot(delta*r2d,'b')
plot(steer*r2d*1.91,'r')
% plot(steer*r2d,'r')
hold off
title('delta')


subplot(246)
hold on
plot(epsi_ref*r2d,'b')
plot(epsi*r2d,'r');
% plot(epsi_2r*r2d,'k')
hold off
title('epsi')

subplot(247)
hold on
% plot(ey_c,'b');
plot(ey,'b');
hold off
title('ey')


subplot(248)
hold on
plot(course_v,'b')
plot(s_center,'r')
% plot(s_ref,'k')
hold off
title('s')


%  4.3134
%%
figure
subplot(221)
hold on
plot(s_ref,'b')
plot(s_center,'r')
hold off
legend('s_ref','calculated')
title('s')

subplot(222)
plot(time)
title('time')


