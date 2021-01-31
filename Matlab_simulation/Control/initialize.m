clc; clear ;

% real s,  computed s, X, Y, Vx, kap, psi

addpath('./reference')
addpath('./map')

name1='reference4000_demo.csv';
name2='reference4000-a135.csv';
name3='reference4000_tokyo.csv';
name4='reference2000_93.csv';


%%

% m=1060;
% Iz=1493.4;
% Caf=33240*2;
% Car=33240*2;
% L=2.3100;
% lf=1.0462;
% lr=1.2638;
% a11=4800*2;
% a12=7.0;
% a21=3720*2;
% a22=10.3;
% mu=1.35;
g=9.81;
dt=1/60;

courseselect=2; %1=demo, 2=tokyo, 3 fuji, 4, brands, 5 361
carselect = 1; % 1 mazda, 2 ttcoup, 3 demioAZWqaQ
% iter_all=30; % choose iteration
iter_all=8;


switch courseselect
    case 1
        course_prefix='demo-';
    case 2
        course_prefix='tokyo-';
        ref_name = 'reference2000_93.csv';
    case 3
        course_prefix='Fuji-';
    case 4
        
        course_prefix='Brands-';
    case 5
        
        course_prefix='361-';
        
end

switch carselect
    case 1
        param_file = 'mazda_params.mat';
        car_prefix = 'mazda_';
    case 2
        param_file = 'ttcoup_params.mat';
        car_prefix = 'ttcoup_';
    case 3
        param_file = 'demio_params.mat';
        car_prefix = 'demio_';
    case 4
        param_file = 'old.mat';
        car_prefix = 'oldparams_';
end
params = load(param_file);
try 
params.a_e = params.ax_max;
params.a_b = params.ax_min;
end
params.g = 9.81;


m = params.m;
Iz = params.Izz;
Caf=33240*2;
Car=33240*2;
L=2.3100;
lf = params.lf;
lr = params.lr;
a11 = params.a11*2;
a12 = params.a12;
a21 = params.a21*2;
a22 = params.a22;
mu = params.mu;
% Caf =  params.C*2;
% Car = params.C*2;
w_Vx = 1;
w_Vy = 0; % orig 0.01;
w_psi_dot = 0;
w_e_psi = 0.09;
w_ey = 400;
w_delta = 0; % orign .05;
r1 = 0.;
r2 = 1;


Q = [w_Vx,w_Vy,w_psi_dot,w_e_psi,w_ey,w_delta];
R = [r1,r2];

params.Q = Q';
params.R = R';
% params.R=[0.1,1]';
% % R=[0.1,1]';
% % Q=[10,0,0,0.1,30,0.1]'; 
% params.Q=[10,0.05,0,0.03,5,1]'; 
% Q=[10,1,0,0.03,100,0.2]'; 
%% main
data0=csvread(ref_name);

Vx_add=-20; 
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
% 1-frame_count
% 2-current_lap_time_msec
% 3-steering
% 4-throttle
% 5-brake
% 6-mass
% 7-wheel_base
% 8-width
% 9length_front
% 10length_rear
% 11course_v
% 12speed_kmph
% 13centerline_diff_angle
% 14centerline_distance
% 15edge_l_distance
% 16edge_r_distance
% 17plane[0]
% 18plane[1]
% 19plane[2]
% 20plane[3]
% 21pos[0]
% 22pos[1]
% 23pos[2]
% 24rot[0]
% 25rot[1]
% 26rot[2]
% 27quaternion[0]
% 28quaternion[1]
% 29quaternion[2]
% 30quaternion[3]
% 31velocity[0]
% 32velocity[1]
% 33velocity[2]
% 34angular_velocity[0]
% 35angular_velocity[1]
% 36angular_velocity[2]
% 37shift_position
% 38is_hit_wall
% 39is_hit_cars
