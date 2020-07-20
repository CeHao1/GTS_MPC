close all 

% 
% load('sv3')
% K=1000;
% % bnd=50;
% course_select=2;
%  [pos,edge,ds]=initialize_centerline(K,course_select); % desample
% iter_all=100;
% 
% xout=sum_out{1,iter_all};
% uout=sum_out{2,iter_all};
% % ds_new=sum_out{4,iter_all};
% kap=sum_out{3,iter_all};

%%




psi=pos(1,:);
Car_ey=xout(5,:);
X=pos(2,:);
Y=pos(3,:);

CarX=X-sin(psi).*Car_ey;
CarY=Y+cos(psi).*Car_ey;
% ds_new=sqrt(diff(CarX).^2+diff(CarY).^2);
% ds_new=[ds_new,ds_new(end)*2-ds_new(end-1)];

psi=pos(1,:)+xout(4,:);


% kap_new=diff(psi)./ds_new(1:end-1);
% kap=[kap_new,kap_new(end)*2-kap_new(end-1)];
%%

psiC=atan2(diff(CarY),diff(CarX));
ind=(psiC<=0.01);
ind(1:100)=0;
psiC(ind)=psiC(ind)+2*pi;

% kap_xy=diff(psiC)./ds_new(2:end-1);
kap_xy=diff(psiC)./ds_new(2:end-1);
dx=extend(diff(CarX));
dy=extend(diff(CarY));
d2x=extend(diff(dx));
d2y=extend(diff(dy));
kap_xy2=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;

Xc=CarX(1);
Yc=CarY(1);
for i=1:length(kap)-1
    Xc(i+1)=Xc(i)+cos(psiC(i))*ds_new(i+1);
    Yc(i+1)=Yc(i)+sin(psiC(i))*ds_new(i+1);
end



% psir=psi(1);
psir=psiC(1);
Xr=CarX(1);
Yr=CarY(1);

for i=1:length(kap)
    psir(i+1)=psir(i)+kap(i)*ds_new(i);
%     psir(i)=psi(i);
    Xr(i+1)=Xr(i)+cos(psir(i))*ds_new(i);
    Yr(i+1)=Yr(i)+sin(psir(i))*ds_new(i);
end




%%
figure
hold on
plot(pos(2,:),pos(3,:),'b--');
plot(edge(1,:),edge(2,:),'r');
plot(edge(3,:),edge(4,:),'g');
% plot(pos(2,1),pos(3,1),'ro');
plot(CarX,CarY,'b--','linewidth',1.5)
plot(CarX(1),CarY(1),'ro');
plot(CarX(end),CarY(end),'r*');
plot(Xr,Yr,'r','linewidth',1.5)
plot(Xc,Yc,'k','linewidth',1.5)
axis equal
hold off

figure
hold on
plot(pos(1,:),'b')
plot(psi,'m')
plot(psir,'r')
plot(psiC,'k')
hold off
legend('cent','epsi+psi_c','kap regenerate','waypoint get')

figure
hold on
plot(pos(5,:))
plot(kap,'r')
plot(kap_xy,'k')
% plot(kap_xy2,'m')
hold off
legend('cent','kap regenerate','waypoint get')
