function [pl,pr,pc,para]=generate_track1()
%% generate initial centerline
interval=0.01;

N=(-10:interval:10)';

pr.X=N;
pr.Y=zeros(length(N),1);

pl.X=N;
% -10:-2
N1=-10:interval:-2;
N1(end)=[];
N2=-2:interval:2;
N3=2:interval:10;
N3(1)=[];
X1=4*ones(length(N1),1);

% X2=(4:-2/(length(N2)-1):2)';
X2=1*sin(-pi/4*N2)'+3;

X3=2*ones(length(N3),1);
pl.Y=[X1;X2;X3];

pl.Y=smooth(pl.Y,50);

%%
pc.X=(pl.X+pr.X)/2;
pc.Y=(pl.Y+pr.Y)/2;

psi=extend(atan2(diff(pc.Y),diff(pc.X)));
dpsi=extend(diff(psi));
ds=extend(sqrt(diff(pc.X).^2+diff(pc.Y).^2));
kap=dpsi./ds;

para.psi=psi;
para.dpsi=dpsi;
para.ds=ds;
para.kap=kap;
% figure
% hold on
% plot(pl.X,pl.Y)
% plot(pr.X,pr.Y)
% plot(pc.X,pc.Y)
% hold on
% 
% axis equal
% 
% figure
% plot(psi)
% title('yaw angle')
% 
% figure
% plot(kap)
% title('curvature')

end