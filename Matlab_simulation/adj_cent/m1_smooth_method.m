clc;clear
close all

r2d=180/pi;

 [pl,pr,pc,para]=generate_track1();

% method 1, smooth
Ns=500;
pc3.X=smooth(pc.X,Ns);
pc3.Y=smooth(pc.Y,Ns);

para3=get_para(pc3);

%% method 2, optimization
% Np=length(pc.X);
% w1=100;
% w2=1;
% % K1=w1*2*para.psi./para.ds;
% % K2=w1./para.ds.^2+w2;
% % K=K1./K2;
% 
% 
% 
% cvx_begin 
% variables d2ey(Np,1) dey(Np,1) ey(Np,1)
% % J=norm(ey+K)
% J=w1*norm(para.dpsi+d2ey./para.ds)+w2*norm(ey)
% minimize(J)
% 
% subject to
% dey(1:Np-1,1)==ey(2:Np,1)-ey(1:Np-1,1)
% dey(Np,1)==dey(Np-1,1)
% d2ey(1:Np-1,1)==dey(2:Np,1)-dey(1:Np-1,1)
% d2ey(Np,1)==d2ey(Np-1,1)
% 
% cvx_end
% 
% dey=cumtrapz(d2ey);
% ey=cumtrapz(dey);
% 
% % get new pos
% pc3.X=pc.X-sin(para.psi).*ey;
% pc3.Y=pc.Y+cos(para.psi).*ey;
% 
% adj_psi=atan2(ey,para.ds);
% % adj_psi=ey./para.ds;
% psi_m2=para.psi+adj_psi;
% para3=get_para(pc3);


% figure
% hold on
% plot(para.dpsi,'b')
% plot(dey./para.ds,'r')
% hold off
% 
% figure
% hold on
% plot(para.psi*r2d,'b')
% plot(psi_m2*r2d,'r')
% plot(adj_psi*r2d,'g')
% plot(para3.psi*r2d,'k');
% hold off
% 
% figure
% hold on
% plot(pl.X,pl.Y,'b')
% plot(pr.X,pr.Y,'b')
% plot(pc.X,pc.Y,'r--')
% plot(pc3.X,pc3.Y,'k')
% hold on
% 
% axis equal

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
