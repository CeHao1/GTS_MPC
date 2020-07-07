function para=get_para(pc)


psi=extend(atan2(diff(pc.Y),diff(pc.X)));
dpsi=extend(diff(psi));
ds=extend(sqrt(diff(pc.X).^2+diff(pc.Y).^2));
kap=dpsi./ds;

para.psi=psi;
para.dpsi=wrapToPi(dpsi);
para.ds=ds;
para.kap=kap;

end