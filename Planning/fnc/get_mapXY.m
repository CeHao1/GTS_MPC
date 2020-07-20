function [CarX,CarY]=get_mapXY(pos,xout)


psi=pos(1,:);
Car_ey=xout(5,:);
X=pos(2,:);
Y=pos(3,:);

CarX=X-sin(psi).*Car_ey;
CarY=Y+cos(psi).*Car_ey;



end