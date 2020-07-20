% clc;clear all;
% close all;
% pos- phi,X,Y,s
% edge- Xl,Yl,Xr,Yr,wl,wrh
% K=1000;
% % bnd=50;
% course_select=2;
%  [pos,edge,ds]=initialize_centerline(K,course_select); % desample

%% X,Y,Xl,Yl,Xr,Yr
centerline=[pos(2,:);pos(3,:);edge(1:4,:)]';
 
filename1 = 'centerline1.csv';
fid1 = fopen(filename1, 'w');
for i=1:length(centerline)
        fprintf(fid1, '%f,%f,%f,%f,%f,%f', centerline(i,:));
        fprintf(fid1, '\n');
end

fclose(fid1);

%%  S,X,Y,Vx,kap,psi
xout=sum_out{1,iter_all};
uout=sum_out{2,iter_all};
ds_new=sum_out{4,iter_all};
s=cumtrapz(ds_new);
% s=[s,s(end)*2-s(end-1)];
kap=sum_out{3,iter_all};
Vx=xout(1,:);
Vy=xout(2,:);
beta=Vy./Vx;

[CarX,CarY]=get_mapXY(pos,xout);
% CarX1=smooth(CarX,20)';
% CarY1=smooth(CarY,20)';
% psi=pos(1,:)+xout(4,:);
psiC=atan2(diff(CarY),diff(CarX));
psi_epsi=pos(1,:)+xout(4,:)+asin(beta);

if courseselect==1
    ind=(psiC<=0.2);
     lp=length(psiC);
    ind(1:400)=0;
    psiC(ind)=psiC(ind)+2*pi;
    psiC=extend(psiC);
    
elseif courseselect==2
    index=(psiC>pi/5);
   lp=length(psiC);
    index(1:round(lp/2))=0;
    psiC(index)=psiC(index)-2*pi;
    psiC=extend(psiC);
end

% psiC1=smooth(psiC,5)';


len=length(psiC);
p1=[psiC+2*pi,psiC,psiC-2*pi];
p2=smooth(p1,10);
psiC1=p2(len+1:2*len)';


% psiC1=smooth_overlap(psiC,20)';

figure
hold on
plot(psiC,'b')
plot(psiC1,'r')
plot(psi_epsi,'k')
hold off


% name={'Vx','Vy','dpsi','epsi','ey','steer','psi'};

reference=[s;CarX;CarY;Vx;kap;psiC1]';
% reference=[s;CarX;CarY;Vx;psiC1;kap]';

filename2 = 'reference_ttcoup.csv';
fid2 = fopen(filename2, 'w');
for i=1:length(reference)
        fprintf(fid2, '%f,%f,%f,%f,%f,%f', reference(i,:));
        fprintf(fid2, '\n');
end

fclose(fid2);


