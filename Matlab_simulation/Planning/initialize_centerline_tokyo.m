function [pos,edge,ds_ori]=initialize_centerline_tokyo(K)
if nargin==0
    K=3000;
end


data0=csvread('centerline_run.csv');
map_data=csvread('centerline_map.csv');
% s, X, Y, Vx, kap, psi

s=data0(:,1)';
X=data0(:,2)';
Y=data0(:,3)';
psi=data0(:,6)';
kap=data0(:,5)';

index=(psi>pi/5);
lp=length(psi);
index(1:round(lp/2))=0;
psi(index)=psi(index)-2*pi;


Xl=map_data(:,3)';
Yl=map_data(:,4)';
Xr=map_data(:,5)';
Yr=map_data(:,6)';

%%
trackwidth=3.75; % a safe gap

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

%%
if nargin==0
    
figure
hold on
plot(s,kap,'b')
plot(s_intp,kap_intp,'r')
hold off

end

end