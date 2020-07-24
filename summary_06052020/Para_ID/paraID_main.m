clear
clc
close all

car = 2;
if car == 1 %% mazda
    namespace = {'mazda_Vxideal.csv', 'mazda_Vx20.csv', 'mazda_Vx40.csv', 'mazda_Vx10.csv','mazda_Vx30.csv'};
elseif car == 2 %% ttcoup
    namespace = {'Vxideal.csv','Vx20.csv', 'Vx40.csv', 'Vx10.csv', 'Vx30.csv'};
elseif car == 3 %% demio
    namespace = {'demio_Vxideal.csv', 'demio_Vx20.csv', 'demio_Vx40.csv', 'demio_Vx10.csv','demio_Vx30.csv'};
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

%% add api given parameters 
paraID.m = params.mass; 
paraID.mu = params.master_mu1;

%% identify C and Izz: 20%-40% of max speed
[d,params] = read_data(namespace{3});
% [d,params] = read_data(namespace{2});
paraID.C = id_C(d, params,plotflag);
paraID.Izz = id_Izz(d,params,paraID,plotflag);


%% identify tanh model: top speed (eg. planned reference)
[d,params] = read_data(namespace{1},1);
[paraID.a11,paraID.a12,paraID.a21,paraID.a22] = ...
    id_tanh(d, params, paraID, plotflag,0);
% if (paraID.a11+paraID.a21)/paraID.m < paraID.mu * g
%     paraID.a11 = (paraID.mu * g)*paraID.m-paraID.a21;
%     (paraID.a11+paraID.a21)/paraID.m
%     [~,~,~,~] = id_tanh(d, params, paraID, 1,paraID.a11);
% end
%% Acceleration limits
% figure()
% plot(d.ay, d.ax,'.')
paraID.ax_max = max(d.ax);
paraID.ax_min = min(d.ax);


%% engine braking and acceleration limit, wind coefficient
% these are parameters we should tune
a_e=1.13; % max ax
a_b=-9.5; % min ax
a_e = paraID.ax_max;
a_b = paraID.ax_min;
% w_wind=2.7e-4;
w_wind=4e-4;

% simulate_v([a_e; a_b; w_wind], d, params, paraID)
opts = optimoptions('fsolve','FunValCheck','on');
tune_params = lsqnonlin (@(tune_params) simulate_v(tune_params, d, params, paraID),...
    [a_e; a_b; w_wind], [0; -Inf; 0], [Inf; 0; 1]);
tune_params = [a_e; a_b; w_wind];

figure()
plot(simulate_vx([a_e; a_b; w_wind], d, params, paraID))
hold on
plot(simulate_vx(tune_params, d, params, paraID))
plot(d.Vx)
legend('orig','tuned','true')
paraID.a_e = tune_params(1);
paraID.a_b = tune_params(2);
paraID.w_wind = tune_params(3);
%% export data
if car == 1
    save('mazda_params', '-struct','paraID')
elseif car ==2 
    save('ttcoup_params', '-struct','paraID')
elseif car == 3 %% demio
    save('demio_params', '-struct','paraID')
end


%% functions
function diff_V = simulate_v(tune_params, d, params, paraID)
    a_e = tune_params(1);
    a_b = tune_params(2);
    w_wind = tune_params(3);
    constraints = get_constraints(d.kap',d.Vx',params.master_mu1,a_e,a_b, paraID);
    Vx = get_Vx2(d,constraints,w_wind, paraID);
    diff_V = sum((Vx-d.Vx).^2);
end

function Vx = simulate_vx(tune_params, d, params, paraID)
    a_e = tune_params(1);
    a_b = tune_params(2);
    w_wind = tune_params(3);
    constraints = get_constraints(d.kap',d.Vx',params.master_mu1,a_e,a_b, paraID);
    Vx = get_Vx2(d,constraints,w_wind, paraID);
end

