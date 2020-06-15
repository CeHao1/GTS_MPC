function [pos,edge,ds_ori]=initialize_centerline_Ugo(loadname)

load(loadname)
len=length(Xc);

X=Xc;
Y=Yc;
% phi=psi;

width=0.4;
interval_new=1/50;

wl=ones(1,len)*width;
wr=wl;

kap_c=get_center_kap(Xc,Yc);

%%
pos=[phi;X;Y;s;kap_c];
pos=[pos,pos(:,1)];
pos(1,end)=pos(1,1)+2*pi;
pos(4,end)=pos(4,end-1)+interval_new;

edge=[Xl;Yl;Xr;Yr;wl;wr];
edge=[edge,edge(:,1)];

% pos(6,1)=interval_new;
ds_ori=interval_new*ones(1,length(pos));
% ds_ori=extend(diff(pos(4,:)));
pos=[pos;ds_ori];

% diff_psi=diff(pos(1,:));
% diff_psi=[diff_psi,diff_psi(1)];
% kap=diff_psi./ds_ori;
% 
% pos(5,:)=kap;


end