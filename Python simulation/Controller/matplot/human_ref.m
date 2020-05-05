function [hs,hX,hY]=human_ref()

i=28;
filename=strcat(num2str(i),'.csv');  % change the name of local path
Dt1=csvread(filename);

% the sequence of different human data might be different. Please change to
% your customized readdata file.

pos=Dt1(:,21:23);
X=pos(:,1);   % x direction: up is positive
Y=-pos(:,3);  % y diection: left is positive


t=Dt1(:,2)/1000; %time ms2s
% dt=1/60;
steer=Dt1(:,3);
thr=Dt1(:,4);
brk=Dt1(:,5);
s=Dt1(:,11);
% V=Dt1(:,12)/3.6;  %kmph 2 mps
velocity=Dt1(:,31:33);
vx=velocity(:,1);
vy=-velocity(:,3);

% len=length(t);

% Cd=Dt1(:,14);
% Ca=Dt1(:,13);
% Dl=Dt1(:,15);
% Dr=Dt1(:,16);

rt=Dt1(:,24:26);
rt2=wrapToPi(pi/2+rt(:,2));  % yaw angle, anti-clockwise is positive
index=(rt2>pi/5);
lp=length(rt2);
index(1:round(lp/2))=0;
rt2(index)=rt2(index)-2*pi;


% q4=Dt1(:,27:30);
av=Dt1(:,34:36);
av2=av(:,2);

% sp=Dt1(:,37);
% hw=Dt1(:,38);

% width=Dt1(1,8);
% m=Dt1(1,6);
% l_f=Dt1(1,9);
% l_r=Dt1(1,10);

Vx=vx.*cos(rt2)+vy.*sin(rt2);
Vy=-vx.*sin(rt2)+vy.*cos(rt2);

%%
dX=extend(diff(X));
dY=extend(diff(Y));
d2X=extend(diff(dX));
d2Y=extend(diff(dY));

kap0=(dX.*d2Y-d2X.*dY)./(dX.^2+dY.^2).^1.5;
kap=smooth(kap0,20);
%%
% xout_ref=[Vx';Vy';av2';X';Y';s';kap'];
% uout_ref=[thr';brk';steer'];
hs=s;
hX=X;
hY=Y;

%%
% figure
% plot(kap)
end
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
