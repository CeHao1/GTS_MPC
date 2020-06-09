close all


% what data to run

% This should be from a very high speed perfromance
% It could be pure pursuit controller tracking the human reference
% while it should not hit the wall or slip heavily.

%%
psi=rt2;

% please change this value according to m3, Izz
Izc=1493.4;

Fyf=(lr*ay*m+Izc*(d2psi))/L/2; % each wheel
Fyr=(lf*ay*m-Izc*(d2psi))/L/2;


x=saf;
y=Fyf;
% this is the initial value, not necessary for a fixed value
b0f=[10000,20];
b1f=tanh_fit2(b0f,x,y);
plot_tanh(x,y,b1f);
sv_mat(1,:)=b1f;

xr=sar;
yr=Fyr;
b0r=[10000,20];
b1r=tanh_fit2(b0r,xr,yr);
sv_mat(2,:)=b1r;
plot_tanh(xr,yr,b1r);


% this is not necessary
% x3=(saf+sar)/2;
% y3=(Fyf+Fyr)/2;
% b03=[10000,20];
% b13=tanh_fit2(b03,x3,y3);
% sv_mat(3,:)=b13;
% plot_tanh(x3,y3,b13);


%%
a11=b1f(1);
a12=b1f(2);
a21=b1r(1);
a22=b1r(2);






