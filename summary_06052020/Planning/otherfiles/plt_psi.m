close all


kap_xy1=extend(diff(psiC))./ds_new(1:end);
dx=extend(diff(CarX));
dy=extend(diff(CarY));
d2x=extend(diff(dx));
d2y=extend(diff(dy));
kap_xy2=(dx.*d2y-d2x.*dy)./(dx.^2+dy.^2).^1.5;


psiS(1)=psiC(2);
psir1(1)=psiC(1);
psir2(1)=psiC(1);

for i=1:length(psiC)-1
    psiS(i+1)=psiS(i)+kap(i)*ds_new(i);
    psir1(i+1)=psir1(i)+kap_xy1(i)*ds_new(i);
    psir2(i+1)=psir2(i)+kap_xy2(i)*ds_new(i);
    
end

%%
% tmp=smooth_overlap(kap_xy1,20);
figure
hold on

plot(kap,'b')
plot(kap_xy1,'r')
hold off

%%

figure
hold on
% plot(psiC,'b')
plot(psiS,'r')
plot(psir1,'k')
plot(psir2,'b')

hold off

figure
plot((psiS-psir1)/pi*180)
