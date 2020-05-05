% clc;clear;
% close

% load('save_data')
load('save_data_np20_500s')


% states: X,Y,yaw, vx,vy,dpsi
% error: epsi,ey
% u: a, dsteer

%%

sum_s=error(:,1);
epsi=error(:,2);
ey=error(:,3);





