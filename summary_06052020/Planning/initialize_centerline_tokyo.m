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
% kap = data0.kap;
Xl = map_data.Xl';
Yl = map_data.Yl';
Xr = map_data.Xr';
Yr = map_data.Yr';
Xc = map_data.Xc';
Yc = map_data.Yc';


% figure
% hold on
% plot(Xc,'k')
% plot(Xl)
% plot(Xr)
% plot(X)
% index=(psi>pi/5);
% lp=length(psi);
% index(1:round(lp/2))=0;
% psi(index)=psi(index)-2*pi;
%%
trackwidth = params.trackwidth; % a safe gap

% wl=ones(1,K)*trackwidth;
% wr=ones(1,K)*trackwidth;


s_intp=intp(s,K);
X_intp=intp(X,K);
Y_intp=intp(Y,K);
psi_intp=intp(psi,K);
% kap_intp=intp(kap,K);
Xl_intp=intp(Xl,K);
Yl_intp=intp(Yl,K);
Xr_intp=intp(Xr,K);
Yr_intp=intp(Yr,K);
Xc_intp = intp(Xc, K);
Yc_intp = intp(Yc,K);

wl = sqrt((Xc_intp - Xl_intp).^2 +(Yc_intp - Yl_intp).^2);
wr = sqrt((Xc_intp - Xr_intp).^2 +(Yc_intp - Yr_intp).^2);


psi_intp = wrapToPi(psi_intp);

diff_psi=wrapToPi(extend(diff(psi)));
intp_diff_psi=intp(diff_psi,K);
ds_ori=extend(diff(s_intp));
kap_intp=intp_diff_psi./ds_ori;

pos=[psi_intp;X_intp;Y_intp;s_intp;kap_intp];
edge=[Xl_intp;Yl_intp;Xr_intp;Yr_intp;wl;wr];

% pos=[pos,pos(:,1)];
% pos(1,end)=pos(1,end)-2*pi;
% pos = wrapToPi(pos);
pos(4,end)=pos(4,end-1)+sqrt((pos(2,end)-pos(2,end-1))^2+(pos(3,end)-pos(3,end-1))^2);

% edge=[edge,edge(:,1)];


pos=[pos;ds_ori];



%%

% pause(0.01)


end