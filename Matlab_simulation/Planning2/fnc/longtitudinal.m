function Vx=longtitudinal(kap,x_last,ds_ori,pos)

% this is the main function to generate longitudinal velocity profile

% initialize the velocity, please ensure that 
% the initial Vx is lower then every points in Vx_max constraints
Vx=ones(1,length(kap))*10;


for i=1:50
    constraints=get_constraints(kap,Vx);
    Vx_new=get_Vx2(Vx,x_last,constraints,ds_ori);
    Vx=(Vx+Vx_new)/2;
    sum_Vx(i,:)=Vx;
    sum_t(i)=sum(ds_ori./Vx);
    bnd=1e-2;
    if i>2 && abs(sum_t(i)-sum_t(i-1))<bnd
        break
    end
    
end

% if you would like to watch the result and plot it,
% please deannotate the following codes

% Vx=get_Vx(Vx,x_last,constraints,ds_ori);
% sum_Vx(i+1,:)=Vx;


% s_pos=pos(4,:);
% figure
% hold on
% h1=plot(s_pos,constraints(1,:),'r','linewidth',1.5);
% h2=plot(s_pos,sum_Vx(1,:)','--','linewidth',1.2);
% plot(s_pos,sum_Vx(1:i,:)','--','linewidth',1.2);
% % plot(sum_Vx(i+1,:)','k','linewidth',2)
% h3=plot(s_pos,Vx,'k','linewidth',2);
% hold off
% axis([-2,max(s_pos)+3,15,50])
% legend([h1,h2,h3],'Limits Vx','Iterative Vx','Final Vx',4)
% title('Velocity Profile','fontsize',20)
% xlabel('Course s (m)','fontsize',20)
% ylabel('Velocity (m/s)','fontsize',20)
% % % 
% % % figure
% % % plot(sum_t)
% pause(0.01)

end

