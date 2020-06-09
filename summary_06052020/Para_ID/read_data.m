function [d,params] = read_data(filename)

data0=csvread(filename);
paraflag=1;

% this is to retrieve the middle part of data, for the start and end have some
% strange points 
interval=round(length(data0)/3000);
if interval==0
    interval=1;
end
cut=length(data0)*1/10;
endcut=length(data0)*9/10;
data=data0(cut:interval:endcut,:);


d.V=data(:,1)/3.6;
d.ax=data(:,4);
d.ay=-data(:,5);   % left is positive
d.sr=data(:,6:9); % slip ratio
d.sa=data(:,10:13); % slip angle
d.wl=data(:,14:17); % wheel load
d.wa=data(:,18:21); % wheel angle
d.steer=data(:,22);
d.thr=data(:,23);
d.brk=data(:,24);
d.erpm=data(:,25);
d.velocity=data(:,26:28);
d.av=data(:,29:31);
d.rt=data(:,32:34);
d.pos=data(:,35:37);
d.course=data(:,38);
d.lap_count=data(:,39);
d.count_num=round(mean(diff(data(:,40))));
d.time=data(:,41)/1000;
d.et=data(:,42); %engine torque
d.sp=data(:,43); %shift position

if paraflag==1 % you may read other states
% 'mass','width','length_front','length_rear','wheel_base','max_ps','engine_rev_limit','master_mu'
    para=data(1,44:54);
    params = table2struct(array2table(para, 'VariableNames', {'mass','width','length_front','length_rear','wheel_base','max_ps','engine_rev_limit','master_mu1', 'master_mu2', 'master_mu3', 'master_mu4'}));
end
%% 
d.srf=(d.sr(:,1)+d.sr(:,2))/2;
d.srr=(d.sr(:,3)+d.sr(:,4))/2;
d.saf=(d.sa(:,1)+d.sa(:,2))/2;
d.sar=(d.sa(:,3)+d.sa(:,4))/2;
d.wlf=(d.wl(:,1)+d.wl(:,2));
d.wlr=(d.wl(:,3)+d.wl(:,4));
d.waf=(d.wa(:,1)+d.wa(:,2))/2;
d.war=(d.wa(:,3)+d.wa(:,4))/2;
d.vx=d.velocity(:,1);
d.vz=d.velocity(:,2);
d.vy=d.velocity(:,3); % 
d.rt2=d.rt(:,2);
d.av2=d.av(:,2);

%%

d.dt=1/60*d.count_num;
% Vx=vx.*sin(rt2)+vy.*cos(rt2);
% Vy=vx.*cos(rt2)-vy.*sin(rt2);

d.Vx=d.vx.*sin(d.rt2)+d.vy.*cos(d.rt2);
d.Vy=-d.vx.*cos(d.rt2)+d.vy.*sin(d.rt2);

d.d2psi=-diff(d.av2)/d.dt;
d.d2psi=[d.d2psi;d.d2psi(end)];

rpm2rads=2*pi/60;

d.erps=d.erpm*rpm2rads;
d.P=d.erps.*d.et;




end