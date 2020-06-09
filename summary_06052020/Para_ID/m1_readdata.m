clc;clear ;
close all

% please read the data according to your data form
% You may use readtable and please set your data 

% I listed too may states while they are not all necessary
% the key states are: X,Y,psi,Vx,Vy,dpsi
% ax,ay
% steer, throttle,brake
% wheel load[4], wheel base, mass

% please make sure the all states should be correct in global coordinate or
% in frenet coordinate. Some states in gym_gts are not consistant with
% frenet coordinate

addpath('./file')
%% readdata
% 1-5,6-8
namespace={'e40+10.csv','e60+20.csv','e100+20.csv','e60+40.csv','e70+45.csv' ...
    ,'tmp02.csv','pp_he_01.csv','pp_test1.csv'};

data0=csvread(namespace{7});
%%
% i show you my method, so I mute them at first
% namespace = 'demio_purepursuit.csv';
% data0=csvread(namespace);
paraflag=1;

% this is to retrive the middle part of data, for the start and end have some
% strange points 
interval=round(length(data0)/3000);
if interval==0
    interval=1;
end
cut=length(data0)*1/10;
endcut=length(data0)*9/10;
data=data0(cut:interval:endcut,:);


V=data(:,1)/3.6;
ax=data(:,4);
ay=-data(:,5);   % left is positive
sr=data(:,6:9); % slip ratio
sa=data(:,10:13); % slip angle
wl=data(:,14:17); % wheel load
wa=data(:,18:21); % wheel angle
steer=data(:,22);
thr=data(:,23);
brk=data(:,24);
erpm=data(:,25);
velocity=data(:,26:28);
av=data(:,29:31);
rt=data(:,32:34);
pos=data(:,35:37);
course=data(:,38);
lap_count=data(:,39);
count_num=round(mean(diff(data(:,40))));
time=data(:,41)/1000;
et=data(:,42); %engine torque
sp=data(:,43); %shift position

if paraflag==1 % you may read other states
% 'mass','width','length_front','length_rear','wheel_base','max_ps','engine_rev_limit','master_mu'
    para=data(1,44:54);
end
%% 
srf=(sr(:,1)+sr(:,2))/2;
srr=(sr(:,3)+sr(:,4))/2;
saf=(sa(:,1)+sa(:,2))/2;
sar=(sa(:,3)+sa(:,4))/2;
wlf=(wl(:,1)+wl(:,2));
wlr=(wl(:,3)+wl(:,4));
waf=(wa(:,1)+wa(:,2))/2;
war=(wa(:,3)+wa(:,4))/2;
vx=velocity(:,1);
vz=velocity(:,2);
vy=velocity(:,3); % 
rt2=rt(:,2);
av2=av(:,2);

%%

dt=1/60*count_num;
% Vx=vx.*sin(rt2)+vy.*cos(rt2);
% Vy=vx.*cos(rt2)-vy.*sin(rt2);

Vx=vx.*sin(rt2)+vy.*cos(rt2);
Vy=-vx.*cos(rt2)+vy.*sin(rt2);

d2psi=-diff(av2)/dt;
d2psi=[d2psi;d2psi(end)];

rpm2rads=2*pi/60;

erps=erpm*rpm2rads;
P=erps.*et;


%%

m=1060.0;
width=1.919;
Iz=1493.4;
% lf=1.0967;
% lr=1.2132 ;
lf=1.0462;
lr=1.2638 ;

L=2.3100;
mps=130.9690;
merpm=7500;
mu=1.35;
g=9.8;
ps2P=735.2941;
mP=mps*ps2P;

