function constraints=get_constraints(kap,Vx,params)
% this function is to get constraints for longitudinal planning
% including Vx max, acc max and acc min



% set mu and g here
mu=params.mu;
% mu=1.40;
g=params.g;
%% Vx
% this ensure curvature is not zero and avoid 1/0
epsilon=1e-10;
kap=kap+epsilon;

% a11 and a21 in tanh model
a11=params.a11;
a21=params.a21;
m=params.m;

% if mu*g is small, select mu*g
ay_m=(a11+a21)/m;
if ay_m>=mu*g;
    ay_m=mu*g;
end

% limit of Vx
Vx_max=sqrt(ay_m./abs(kap));

%% ax
ay=Vx.^2.*abs(kap);

ax_max0=(ay_m^2-ay.^2);

ax_max0(ax_max0<0)=0;

ax_max=sqrt(ax_max0);
ax_min=-ax_max;

a_e=1.3; %engine limit
% a_e=1.35; %engine
a_b=-9.5; % braking limit
ax_max(ax_max>a_e)=a_e;
ax_min(ax_min<a_b)=a_b;
% ax_max=ones(1,length(kap))*a_e;
% ax_min=ones(1,length(kap))*a_b;


%%
constraints=[Vx_max;ax_max;ax_min];


end