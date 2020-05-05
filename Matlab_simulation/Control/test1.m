close all

s=sum_s(range);
cX=map_data(:,1);
cY=map_data(:,2);

vX=X(range);
vY=Y(range);

ref=[cX,cY];

for i=1:length(vX)
    ey_c(i)=find_near_point([vX(i),vY(i)],ref);
    
    
end

plot(ey_c)