function Vx_update=get_Vx2(d,constraints,w_wind,paraID)
% this function is to use forward and backward propagation to calculate Vx

Vx=d.Vx';
Vy=d.Vy';
ds_ori=d.ds';
dpsi=d.dpsi';
delta=d.steer';

st=length(Vx);

% first of all, we need to overlap the track data to avoid edge effect 
% so, to repeat matrix to three fold

constraints=repmat(constraints,1,3);
Vx=repmat(Vx,1,3);
Vy=repmat(Vy,1,3);
ds_ori=repmat(ds_ori,1,3);
dpsi=repmat(dpsi,1,3);
delta=repmat(delta,1,3);

% calculate lateral force of front tire
% lf=1.0462;
% a11=4800*2;
% a12=7.0;
% m=1060;

% a11=5.566e3;
% a12=15.80;
% lf=0.9304;
% m=1355.2;
a11 = paraID.a11;
a12 = paraID.a12;
lf = paraID.lf;
m = paraID.m;

alpf=delta-atan2(Vy+lf*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Dd=(Fyf.*sin(delta))/m-dpsi.*Vy;

% w_wind=2.7e-4;
% w_wind=0;
a_wind=w_wind*Vx.^2;

Np=length(Vx);

%%
% retrive constraints
Vmax=constraints(1,:);
amax=constraints(2,:);
amin=constraints(3,:);

% forward
V1(1)=Vmax(1);

 a_ac=amax-Dd-a_wind;
 a_dc=amin-Dd-a_wind;

for i=1:Np-1
    V1(i+1)=min([Vmax(i+1),sqrt(2*a_ac(i)*ds_ori(i)+V1(i)^2)]);
end
% V1=abs(V1);

% backward
% if Vk>Vk+1, decelerate, but no more than vm
V2(Np)=Vmax(Np);
for i= Np:-1:2   
    V2(i-1)=min([Vmax(i-1),sqrt(-2*a_dc(i)*ds_ori(i-1)+V2(i)^2)]);
end
% V2=abs(V2);

Va=min([V1;V2]);
Vx_update=Va(st+1:2*st);
%%
% if you want to watch the result, please deannotate the following figure
rg=(st+1:2*st);
% % 
% figure
% hold on;
% h1=plot(Vmax(rg),'b','linewidth',1.5);
% h2=plot(V1(rg),'r','linewidth',1.5);
% h3=plot(V2(rg),'linewidth',1.5,'Color',[60,179,113]/255);
% h4=plot(Va(rg),'k','linewidth',2);
% h5=plot(Vx(rg),'m','linewidth',2);
% % xlabel('distance (m)');
% % ylabel('velocity (m/s)');
% 
% xlabel('waypoints (m)','fontsize',15);
% ylabel('Vx (m/s)','fontsize',15);
% h=legend([h1,h2,h3,h4,h5],'Vx limit','forward','backward','result','human');
% set(h,'fontsize',12)
% 
% hold off;
% axis([0,length(rg),0,100])
% 
% 
% figure
% hold on
% plot(amax,'r')
% plot(amin,'b')
% plot(a_ac,'r*')
% plot(a_dc,'b*')
% hold off
% 
% pause(0.01)
% close all

end