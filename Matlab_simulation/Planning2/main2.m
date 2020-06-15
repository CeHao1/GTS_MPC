
%%------------------- Model-based trajectory planning ------------------------
% Author: Ce Hao
% Time: 06/04/2020
% Description:
% To minimize curvature in the track by three steps: velocity profile, 
% path planning and curvature update

% we provide two examples, the first is a easy demo, and the second is
% tokyo expressway central outer loop in GT Sport platform

%--------------------------- about plot figure-----------------------------
% if choose demo, run plt1_demo, or run plt2_tokyo
% and to plot saved data, use plt_all instead
% the rest of plt function is to watch different states, you may choose
% whatever you want.

% function save_reference is to save the data as .csv

%%
% pos- phi,X,Y,s
% edge- Xl,Yl,Xr,Yr,wl,wrh
% if ~exist('K')
clc;clear ;
close all;
addpath('./fnc');
addpath('./csv');
    
K=2000; % choose waypoints
courseselect=2; %1=demo, 2=tokyo
iter_all=30; % choose iteration
% iter_all=15;

switch courseselect
    case 1
    [pos,edge,ds_ori]=initialize_centerline_demo(K); 
    name_prefix='demo-';
    case 2
    [pos,edge,ds_ori]=initialize_centerline_tokyo(K);
    name_prefix='tokyo-';
end
  
 %    pos=[phi;X;Y;s,kap];
x_last=zeros(6,length(pos));

kap=pos(5,:);

% end

%%
% if you'd like to watch the wait bar, please deannotate them
% h=waitbar(0,'running!');


for iter=1:iter_all
    iter
    tic
%     str=['running...',num2str(((iter-1)/iter_all)*100),'%'];
%     waitbar(((iter-1)/iter_all),h,str);
    
    
if exist('xout')
    ds_ori=ds_new;
%     x_last=xout;
% this is the damping effect, so there is 1/2
    x_last=(xout+x_last)/2;
end


Vx=longtitudinal(kap,x_last,ds_ori,pos);
% x_last(1,:)=Vx;
% damping as well
x_last(1,:)=(Vx+x_last(1,:))/2;

V=sqrt(x_last(1,:).^2+x_last(2,:).^2);
t(iter)=sum(ds_ori./V);

[xout,uout]=lateral(pos,edge,x_last,ds_ori);
[kap,ds_new]=get_kap(pos,xout);


sum_out{1,iter}=xout;
% sum_out{1,iter}=x_last;
sum_out{2,iter}=uout;
sum_out{3,iter}=kap;
sum_out{4,iter}=ds_new;
toc

end
sum_out{5,1}=t;

% close(h);


% save result
name=strcat(name_prefix,num2str(K),'-',num2str(iter_all));
save(name,'sum_out','pos','edge','iter_all','kap','ds_ori','courseselect')
msgbox('end')



