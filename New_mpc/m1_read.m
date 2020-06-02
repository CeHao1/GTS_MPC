close all
clc
clear

data0=csvread('r3.csv');


pos=data0(:,1:3);
rot=data0(:,4:6);
velocity=data0(:,7:9);
av=data0(:,10:12);
time=data0(:,13)/1000;
fc=data0(:,14);
course_v=data0(:,15);
sa=data0(:,16:19);
delta=data0(:,20);
acc=data0(:,21);

X=data0(:,22);
Y=data0(:,23);
psi=data0(:,24);
Vx=data0(:,25);
Vy=data0(:,26);
dpsi=data0(:,27);
ey=data0(:,28);
epsi=data0(:,29);
%%

vx=velocity(:,1);
vy=velocity(:,3);

Vx2=vx.*cos(psi)+vy.*sin(psi);
Vy2=-vx.*sin(psi)+vy.*cos(psi);





