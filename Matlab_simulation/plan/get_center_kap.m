function kap=get_kap(x,y)


dx=extend(diff(x));
dy=extend(diff(y));

d2x=extend(diff(dx));
d2y=extend(diff(dy));

kap=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;


end