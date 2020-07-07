clc;clear
close all

r2d=180/pi;

 [pl,pr,pc,para]=generate_track1();

% method 1, smooth
% Ns=60;
% pc2.X=smooth(pc.X,Ns);
% pc2.Y=smooth(pc.Y,Ns);
% 
% para2=get_para(pc2);

%% method 2, optimization
pc3=pc;
para3=para;

Np=length(pc3.X);
w1=40;
w2=0.1;

for i=1:5

cvx_begin quiet
variables d2ey(Np,1) dey(Np,1) ey(Np,1)
% J=norm(ey+K)
J=w1*norm(para3.dpsi+d2ey./para3.ds)+w2*norm(ey)
minimize(J)

subject to
dey(1:Np-1,1)==ey(2:Np,1)-ey(1:Np-1,1)
dey(Np,1)==dey(Np-1,1)
d2ey(1:Np-1,1)==dey(2:Np,1)-dey(1:Np-1,1)
d2ey(Np,1)==d2ey(Np-1,1)

cvx_end

dey=cumtrapz(d2ey);
ey=cumtrapz(dey);

% get new pos
pc3.X=pc3.X-sin(para3.psi).*ey;
pc3.Y=pc3.Y+cos(para3.psi).*ey;

adj_psi=atan2(ey,para3.ds);
% adj_psi=ey./para.ds;
psi_m2=para3.psi+adj_psi;
para3=get_para(pc3);


end
%%
plotflag=1;

if plotflag==1

figure
hold on
plot(pl.X,pl.Y,'b')
plot(pr.X,pr.Y,'b')
plot(pc.X,pc.Y,'r--')
plot(pc3.X,pc3.Y,'k')
hold on

axis equal


figure
hold on
plot(para.psi,'b')
plot(para3.psi,'r')
hold off
title('psi')

figure
hold on
plot(para.kap,'b')
plot(para3.kap,'r')
hold off
title('kap')

figure
plot(para3.ds)

end
