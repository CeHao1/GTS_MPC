r2d=180/pi;

index_htw=index_htw0(7:9);

close all
addpath('./map')
name2p='centerline2_tokyo.csv';
map_data=csvread(name2p);

r0=readtable('reference_60.csv');
reference=table2array(r0);

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
h4=plot(X(1),Y(1),'ro','linewidth',2);

plot(map_data(:,1),map_data(:,2),'b--');
plot(map_data(:,3),map_data(:,4),'r--');
plot(map_data(:,5),map_data(:,6),'m--');

h3=plot(X(index_htw),Y(index_htw),'k*','linewidth',2);
% plot(X(index_htw),Y(index_htw),'ro','linewidth',2);

hold off
axis equal
plotdist=100;
axis([min(map_data(:,1))-plotdist,max(map_data(:,1))+plotdist,min(map_data(:,2))-plotdist,max(map_data(:,2))+plotdist]);
title('X-Y position','fontsize',15)
h=legend([h1,h2,h4,h3],'reference','real path','start point','collision');
set(h,'fontsize',15)


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


[hs,hX,hY]=human_ref();
ey_h=[];
for i=1:length(hX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_h(i),~,~,~]=find_near_point_0([hX(i),hY(i)],ref);  
end


figure
hold on
h1=plot(s,ones(1,length(s))*3.75,'k-','linewidth',2);
plot(s,-ones(1,length(s))*3.75,'k-','linewidth',2);
h2=plot(s,ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255);
plot(s,-ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255)

h3=plot(hs,ey_h,'-','linewidth',2,'Color',[60,179,113]/255);
h4=plot(rs,ey_r,'b-','linewidth',2);
% h5=plot(s,ey_c,'-','linewidth',2,'color',[255,69,0]/255);
h5=plot(s_center,ey_c,'-r','linewidth',2);

plot(s_center(index_htw),ey_c(index_htw),'k*','linewidth',2);

hold off
title('ey of centerline','fontsize',15)
legend([h1,h2,h3,h4,h5],'\fontsize {15} Width of Track','\fontsize {15} Width with Safe Gap','\fontsize {15} Human','\fontsize {15} Treajectory Planning','\fontsize {15} MPC')
xlabel('Course / m','fontsize',20)
ylabel('Lateral path deviation / m','fontsize',20)

end

%%
href=human_ref2();


figure

hold on
h1=plot(course_v,referenceVx,'r','linewidth',1.5);
h2=plot(s,Vx,'b','linewidth',1.5);

h3=plot(href.s,href.Vx,'linewidth',1.5,'color',[60,179,113]/255);

plot(s(index_htw),Vx(index_htw),'k*','linewidth',1.5);

h=legend([h1,h2,h3],'Reference','MPC','Human');

hold off
set(h,'fontsize',15)
title('Vx')








