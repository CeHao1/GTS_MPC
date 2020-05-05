function [dist,psi_c,psi_v,psi_j]=find_near_point(pt,ref)

X=pt(1);
Y=pt(2);
[a,b]=size(ref);
if a==2
rX=ref(1,:);
rY=ref(2,:);
elseif b==2
rX=ref(:,1);
rY=ref(:,2);
end

% first find nearest point a-1, a, a+1
t1=(X-rX).^2+(Y-rY).^2;
[v1,n1]=min(t1);
n0=n1-1;
n2=n1+1;
if n1==1
    n0=length(rX-2);
elseif n1==length(rX)
    n2=2;
end

% interpolate with 0.01 distance
p3=0:2;
p1=0:0.001:2;
X_inpt=spline(p3,[rX(n0),rX(n1),rX(n2)],p1);
Y_inpt=spline(p3,[rY(n0),rY(n1),rY(n2)],p1);

% find the nearest again
t2=(X-X_inpt).^2+(Y-Y_inpt).^2;
[v2,n3]=min(t2);

% angle
psi_c=atan2(rY(n1)-rY(n0),rX(n1)-rX(n0));
psi_v=atan2(Y-Y_inpt(n3),X-X_inpt(n3));
psi_j=wrapToPi(psi_v-psi_c);

dist=sqrt(v2)*sign(psi_j);


end




