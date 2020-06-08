close all

if ~exist('xout')
    xout=sum_out{1,end};
    uout=sum_out{2,end};
end

eyr(1)=xout(5,1);



for i=1:length(kap)
    index=i;
%     Vx=xout(1,index);
%     Vy=xout(2,index);
%     V=sqrt(Vx^2+Vy^2);
%     ds_now=ds_ori(index);
%     dt=ds_now/V;
%     
%     epsi_last=x_last(4,index);
%     epsi=xout(4,index);
    Vx=xout(1,index);
    Vy=xout(2,index);

    dey=Vx*sin(epsi)+Vy*cos(epsi);
    eyr(i+1)=eyr(i)+dey*dt;
    
end



% eyr(1)=xout(5,1);
% 
% for i=1:length(epsi)
%     epsi_last=x_last(4,i);
%     epsi=xout(4,i);
%     Vx=x_last(1,i);
%     Vy=x_last(2,i);
%     ey=xout(5,i);
%     
%     dt=ds_ori(i)/sqrt(Vx^2+Vy^2);
%     
%     Vx=xout(1,i);
%     Vy=xout(2,i);
% 
%     dey=Vx(i)*sin(epsi)+Vy*cos(epsi);
%     eyr(i+1)=eyr(i)+dey*dt;
%     
% end

%%

figure
hold on
plot(xout(5,:),'b')
plot(eyr,'r')
hold off





