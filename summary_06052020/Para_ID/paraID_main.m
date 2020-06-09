clear
clc
close all

namespace = {'demio_purepursuit.csv','demio40sine.csv', 'demio40.csv', 'demio10.csv', 'demio30.csv'};
plotflag = 1;

g = 9.81;
%% identify lf and lr: low speed id
[d,params] = read_data(namespace{4});
m = params.mass;
L = params.wheel_base;
paraID.lf=mean(d.wlr/(m*g))*L;
paraID.lr=mean(d.wlf/(m*g))*L;


%% identify C and Izz: 20%-40% of max speed
[d,params] = read_data(namespace{5});
paraID.C = id_C(d, params,plotflag);
paraID.Izz = id_Izz(d,params,paraID,plotflag);

%% identify tanh model: top speed (eg. planned reference)
[d,params] = read_data(namespace{1});
[paraID.a11,paraID.a12,paraID.a21,paraID.a22] = ...
    id_tanh(d, params, paraID, plotflag);

%% add given parameters and export
paraID.m = params.mass; 
