function Vx_update=get_Vx2(Vx,x_last,constraints,ds_ori,params)
% this function is to use forward and backward propagation to calculate Vx

lf = params.lf;
a11 = params.a11;
a12 = params.a12;
m = params.m;
w_wind = params.w_wind;

st=length(Vx);
% first of all, we need to overlap the track data to avoid edge effect 
% so, to repeat matrix to three fold
Vx=repmat(Vx,1,3);
x_last=repmat(x_last,1,3);
constraints=repmat(constraints,1,3);
ds_ori=repmat(ds_ori,1,3);

% Vx=x_last(1,:);
Vy=x_last(2,:);
dpsi=x_last(3,:);
delta=x_last(6,:);

% calculate lateral force of front tire

alpf=delta-atan2(Vy+lf*dpsi,Vx);
Fyf=a11*tanh(a12*alpf);
Dd=(Fyf.*sin(delta))/m-dpsi.*Vy;


a_wind=w_wind*Vx.^2;

Np=length(x_last);

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

% figure
% hold on;
% plot(Vmax,'b');
% plot(V1,'r');
% plot(V2,'g');
% plot(Va,'k','linewidth',2);
% xlabel('distance (m)');
% ylabel('velocity (m/s)');
% 
% hold off;
% axis([1000,2000,20,80])
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
% close all

end