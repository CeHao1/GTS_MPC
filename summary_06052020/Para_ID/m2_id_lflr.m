% low speed 



% What data shoule we use 
% On flat ground and static or very-low speed 

% to identify lf and lr
% L=2.3100;
L = para(5);
m = para(1);

lf=mean(wlr/(m*9.8))*L;
lr=mean(wlf/(m*9.8))*L;