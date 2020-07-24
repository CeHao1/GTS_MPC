function kap=get_kap(CarX,CarY)


Nsm=20;
dx=smooth(extend(diff(CarX)),Nsm);
dy=smooth(extend(diff(CarY)),Nsm);
d2x=smooth(extend(diff(dx)),Nsm);
d2y=smooth(extend(diff(dy)),Nsm);
kap0=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;
kap=kap0';


end