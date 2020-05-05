clc; clear ;

% real s,  computed s, X, Y, Vx, kap, psi
run loadfile

addpath('./reference')
addpath('./map')

name1='reference4000_demo.csv';
name2='reference4000-a135.csv';
name3='reference4000_tokyo.csv';

name4='reference2000_93.csv';

data0=csvread(name4);

Vx_add=0; 
% s, X, Y, Vx, kap, psi
reference=data0;

% X,Y,psi,Vx,Vy,dpsi
reference(:,4)=reference(:,4)+Vx_add;

% init=[reference(1,2:3),reference(1,6),reference(1,4),0,0];
init=[reference(1,2:3),reference(1,6),15,0,0];

% figure
% plot(reference(:,1),reference(:,5))
% figure
% plot(reference(:,2),reference(:,3))
%%

m=1060;
Iz=1493.4;
Caf=33240*2;
Car=33240*2;
L=2.3100;
lf=1.0462;
lr=1.2638;
a11=4800*2;
a12=7.0;
a21=3720*2;
a22=10.3;
mu=1.35;
g=9.8;
dt=1/60;

