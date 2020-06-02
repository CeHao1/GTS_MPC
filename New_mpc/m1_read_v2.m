close all
clc
clear

name='MPCresults.csv';
name2='MPCresults_93.15.csv';
% data0=csvread('r1.csv');
data0=csvread(name);

pos=data0(:,1:3);
rot=data0(:,4:6);
velocity=data0(:,7:9);
av=data0(:,10:12);
time=data0(:,13)/1000;
fc=data0(:,14);
course_v=data0(:,15);

steer=data0(:,16);
thr=data0(:,17);
brk=data0(:,18);

% sa=data0(:,16:19);


delta=data0(:,23);
acc=data0(:,24);

X=data0(:,25);
Y=data0(:,26);
psi=data0(:,27);
Vx=data0(:,28);
Vy=data0(:,29);
dpsi=data0(:,30);
ey=data0(:,31);
epsi=data0(:,32);
s_center=data0(:,33);
%%

vx=velocity(:,1);
vy=velocity(:,3);

Vx2=vx.*cos(psi)+vy.*sin(psi);
Vy2=-vx.*sin(psi)+vy.*cos(psi);





