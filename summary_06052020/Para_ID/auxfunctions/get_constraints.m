function constraints=get_constraints(kap,Vx,mu,a_e,a_b,paraID)
% this function is to get constraints for longitudinal planning
% including Vx max, acc max and acc min


% set mu and g here
% mu=1.3;
% mu=1.40;
g=9.8;
% n_ori = length(kap);
% kap = [kap kap kap];
% Vx = [Vx Vx Vx];
%% Vx
% this ensure curvature is not zero and avoid 1/0
epsilon=1e-10;
kap=kap+epsilon;

% a11 and a21 in tanh model

% a11=4800*2;
% a21=3720*2;
% m=1060;


% a11=5.566e3*2;
a11 = paraID.a11;
% a21=6.1919e3*2;
a21 = paraID.a21;
% m=1355.2;
m = paraID.m;

% if mu*g is small, select mu*g
% ay_m=(a11+a21)/m;
% if ay_m>=mu*g
%     ay_m=mu*g;
% end
ay_m=mu*g;

% limit of Vx
Vx_max=sqrt(ay_m./abs(kap));

%% ax
ay=Vx.^2.*abs(kap);

ax_max0=(ay_m^2-ay.^2);

ax_max0(ax_max0<0)=0;

ax_max=sqrt(ax_max0);
ax_min=-ax_max;

% a_e=1.3; %engine limit
% a_e=1.35; %engine
% a_b=-8.0; % braking limit
ax_max(ax_max>a_e)=a_e;
ax_min(ax_min<a_b)=a_b;
% ax_max=ones(1,length(kap))*a_e;
% ax_min=ones(1,length(kap))*a_b;


%%
% Vx_max = Vx_max(n_ori+1:end-n_ori);
% ax_max = ax_max(n_ori+1:end-n_ori);
% ax_min = ax_min(n_ori+1:end-n_ori);
constraints=[Vx_max;ax_max;ax_min];


end