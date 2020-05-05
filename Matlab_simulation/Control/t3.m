close all

dey2=extend(diff(ey)/dt);
figure
hold on
plot(dey,'b')
plot(dey2,'r')
hold off

figure
plot(beta_epsi,dey2,'.')

