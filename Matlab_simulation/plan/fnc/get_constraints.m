function constraints=get_constraints(kap,Vx)

mu=0.45;
% mu=1.40;
g=9.8;
%% Vx
epsilon=1e-10;
kap=kap+epsilon;
% a11=4800*2;
% a21=3720*2;
% m=1060;
% 
% ay_m=(a11+a21)/m;
% if ay_m>=mu*g;
%     ay_m=mu*g;
% end
ay_m=mu*g;

Vx_max=sqrt(ay_m./abs(kap));

%% ax
ay=Vx.^2.*abs(kap);

ax_max0=(ay_m^2-ay.^2);

ax_max0(ax_max0<0)=0;

ax_max=sqrt(ax_max0);
ax_min=-ax_max;

a_e=2; %engine
a_b=-2;
ax_max(ax_max>a_e)=a_e;
ax_min(ax_min<a_b)=a_b;



%%
constraints=[Vx_max;ax_max;ax_min];


end