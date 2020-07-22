clear
clc
close all

car = 1;
if car == 1 %% mazda
    namespace = {'mazda_Vxideal.csv', 'mazda_Vx20.csv', 'mazda_Vx40.csv', 'mazda_Vx10.csv','mazda_Vx30.csv'};
elseif car == 2 %% ttcoup
    namespace = {'Vxideal.csv','Vx20.csv', 'Vx40.csv', 'Vx10.csv', 'Vx30.csv'};
end
% namespace = {'demio_purepursuit.csv','demio40sine.csv', 'demio40.csv', 'demio10.csv', 'demio30.csv'};
plotflag = 1;

g = 9.81;
%% identify lf and lr: low speed id
[d,params] = read_data(namespace{4});
m = params.mass;
L = params.wheel_base;
paraID.lf=mean(d.wlr/(m*g))*L;
paraID.lr=mean(d.wlf/(m*g))*L;


%% identify C and Izz: 20%-40% of max speed
[d,params] = read_data(namespace{3});
paraID.C = id_C(d, params,plotflag);
paraID.Izz = id_Izz(d,params,paraID,plotflag);


%% identify tanh model: top speed (eg. planned reference)
[d,params] = read_data(namespace{1},1);
[paraID.a11,paraID.a12,paraID.a21,paraID.a22] = ...
    id_tanh(d, params, paraID, plotflag);

%% Acceleration limits
% figure()
% plot(d.ay, d.ax,'.')
ax_max = max(d.ax);
ax_min = min(d.ax);

%% engine braking and acceleration limit


%% add api given parameters and export
paraID.m = params.mass; 
paraID.mu = params.master_mu1;

if car == 1
    save('mazda_params', '-struct','paraID')
elseif car ==2 
    save('ttcoup_params', '-struct','paraID')
end