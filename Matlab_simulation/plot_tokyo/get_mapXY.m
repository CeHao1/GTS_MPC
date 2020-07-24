function d=get_mapXY(d)
pos=d.pos;
xout=d.xout;

psi=pos(1,:);
Car_ey=xout(5,:);
X=pos(2,:);
Y=pos(3,:);

CarX=X-sin(psi).*Car_ey;
CarY=Y+cos(psi).*Car_ey;

d.CarX=CarX;
d.CarY=CarY;

end