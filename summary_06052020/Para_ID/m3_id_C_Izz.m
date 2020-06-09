close all

% what data to run

% relatively low speed, about 20%-40% of the maximum speed
% along the centerline and avoid sharp turnning
% make sure that the figure alpha to Fy is proportional and linear, or to
% decelerate the Vx

% identify C and Izz, please set Izz in m4 as the result of m3

%% C
% method1
cvx_begin
variable C
J=ay*m/2-C*(saf+sar)
minimize(norm(J))
cvx_end

% method2
% inx=(saf+sar)/2;
% outy=ay*m/2;
% b0c=[10000,20];
% b1c=tanh_fit2(b0c,inx,outy);
% 
% sim2=b1c(1)*tanh(b1c(2)*inx);

figure
hold on
plot((saf+sar),ay*m/2,'b.')
% plot(inx*2,sim2,'r','linewidth',2)
plot((saf+sar),(saf+sar)*C,'k','linewidth',1.5)
hold off
title(strcat('C_\alpha : ',num2str(C)),'fontsize',15)
xlabel('\alpha','fontsize',15)
ylabel('Fy','fontsize',15)
%% Izz
xx=2*d2psi/2;
yy=C*L*(saf-sar)-ay*m*(lr-lf)/2;
% yy=(b1c(1)*tanh(b1c(2)*saf)   -   b1c(1)*tanh(b1c(2)*sar) )*L-ay*m*(lr-lf)/2;

cvx_begin
variable Izz
J2=xx*Izz-yy
minimize(norm(J2))
cvx_end

figure
hold on
plot(xx,yy,'b.')
plot(xx,xx*Izz,'r','linewidth',2)
hold off
title(strcat('Izz :',num2str(Izz)),'fontsize',15)
% xlabel('$\ddot \psi $','Interpreter','latex','fontsize',15) 
ylabel('M','fontsize',15)




