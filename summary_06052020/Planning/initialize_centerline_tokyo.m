function [pos,edge,ds_ori]=initialize_centerline_tokyo(K,params)
if nargin==0
    K=3000;
end

%% load data
data0=readtable('centerline_run.csv');
map_data=readtable('centerline_map.csv');
s = data0.s';
X = data0.X';
Y = data0.Y';
psi = data0.psi';
kap = data0.kap;
Xl = map_data.Xl';
Yl = map_data.Yl';
Xr = map_data.Xr';
Yr = map_data.Yr';

index=(psi>pi/5);
lp=length(psi);
index(1:round(lp/2))=0;
psi(index)=psi(index)-2*pi;
%%
trackwidth = params.trackwidth; % a safe gap

wl=ones(1,K)*trackwidth;
wr=ones(1,K)*trackwidth;

s_intp=intp(s,K);
X_intp=intp(X,K);
Y_intp=intp(Y,K);
psi_intp=intp(psi,K);
kap_intp=intp(kap,K);
Xl_intp=intp(Xl,K);
Yl_intp=intp(Yl,K);
Xr_intp=intp(Xr,K);
Yr_intp=intp(Yr,K);

pos=[psi_intp;X_intp;Y_intp;s_intp;kap_intp];
edge=[Xl_intp;Yl_intp;Xr_intp;Yr_intp;wl;wr];

pos=[pos,pos(:,1)];
pos(1,end)=pos(1,end)-2*pi;
pos(4,end)=pos(4,end-1)+sqrt((pos(2,end)-pos(2,end-1))^2+(pos(3,end)-pos(3,end-1))^2);

edge=[edge,edge(:,1)];

ds_ori=extend(diff(pos(4,:)));
pos=[pos;ds_ori];

diff_psi=diff(pos(1,:));
diff_psi=[diff_psi,diff_psi(1)];
kap=diff_psi./ds_ori;

pos(5,:)=kap;



end