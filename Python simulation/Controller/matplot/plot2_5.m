close all
 


%  range=1:length(t);


%% Vx
figure
V=sqrt(Vx.^2+Vy.^2);
hold on
plot(t(range),Vx_ref(range),'b','linewidth',2)
plot(t(range),V(range),'r','linewidth',2)
hold off
% title(strcat('Vx, laptime: ',num2str(laptime)),'fontsize',15)
legend('Trajectory Planning','MPC')
xlabel('Course / m','fontsize',15)
ylabel('Velocity m/s','fontsize',15)

% %% ey
% figure
% plot(sum_s(range),ey(range),'linewidth',1.5)
% title('ey','fontsize',15)
% xlabel('Course (m)','fontsize',15)
% ylabel('e_y (m/s)','fontsize',15)



%%


% centerline,ey
s=sum_s(range);
cX=map_data(:,1);
cY=map_data(:,2);

vX=X(range);
vY=Y(range);

ref=[cX,cY];
ref1=ref;
ref2=[ref(end-10:end,:);ref];
ey_c=[];
for i=1:length(vX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_c(i),psi_c(i),psi_v(i),psi_j(i)]=find_near_point([vX(i),vY(i)],ref);  
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
    [ey_r(i),~,~,~]=find_near_point([rX(i),rY(i)],ref);  
end

[hs,hX,hY]=human_ref();
ey_h=[];
for i=1:length(hX)
    if i<500
        ref=ref2;
    else
        ref=ref1;
    end
    [ey_h(i),~,~,~]=find_near_point([hX(i),hY(i)],ref);  
end
%%

ey_h(1:50)=smooth(ey_h(1:50),40);
ey_c(1:50)=smooth(ey_c(1:50),40);

figure
hold on
h1=plot(s,ones(1,length(s))*3.75,'k-','linewidth',2);
plot(s,-ones(1,length(s))*3.75,'k-','linewidth',2);
h2=plot(s,ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255);
plot(s,-ones(1,length(s))*3.25,'-','linewidth',2,'color',[138,43,226]/255)

h3=plot(hs,ey_h,'--','linewidth',2,'Color',[60,179,113]/255);
h4=plot(rs,ey_r,'b-','linewidth',2);
% h5=plot(s,ey_c,'-','linewidth',2,'color',[255,69,0]/255);
h5=plot(s,ey_c,'-r','linewidth',2);

hold off
% title('ey of centerline','fontsize',15)
legend([h1,h2,h3,h4,h5],'\fontsize {15} Width of Track','\fontsize {15} Width with Safe Gap','\fontsize {15} Human','\fontsize {15} Treajectory Planning','\fontsize {15} MPC')
xlabel('Course / m','fontsize',20)
ylabel('Lateral path deviation / m','fontsize',20)

% \fontsize {15} 
%%
% dVy=extend(diff(Vy)/dt);
% ay2=dVy+Vx.*dpsi;
% 
% theta=0:0.01:2*pi;
% fm=mu*g;
% cirX=fm*cos(theta);
% cirY=fm*sin(theta);
% ax=input(:,1);
% 
% figure
% hold on
% plot(cirX,cirY,'r','linewidth',2)
% plot(-ay(range),ax(range),'b.')
% hold off
% 
% axis equal
% title('g-g diagram','fontsize',15)
% xlabel('ay','fontsize',15)
% ylabel('ax','fontsize',15)




