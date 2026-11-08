function [c,z0] = hcf_certificate(p,R,nu)
% Original manuscript's scalar sufficient certificate, n=2.
% Bounds cover all t>=0 and the entire verification output rectangle.
c.R=R; c.nu=nu; c.mu10=1; c.mu20=1.5; c.P2=1.5; c.m2=3;
c.r=[R^2*p.p1/(4*p.eta1),R^2*p.p2/(4*p.eta1)];
c.Omega1=[-1.2-c.r(1)-nu,1.3+c.r(2)+nu];
xa=max(abs(c.Omega1));
hmax=[c.Omega1(2)+1.2,1.3-c.Omega1(1)];
hmin=[c.Omega1(1)+1,0.9-c.Omega1(2)];
comp=exp(-hmin)./[p.p1,p.p2]; habs=max(abs(hmin),abs(hmax));
c.whole_box_gap=1.9*p.eta1-0.6-sum(comp);
c.sharing_gap=1.9*p.eta1-0.6-sum(exp(c.r+nu)./[p.p1,p.p2]);
c.K1=max([0.1*xa^2+0.6+p.ke*(xa+2),...
    0.1*xa^2+0.2+comp(1)+p.eta1*habs(1),...
    0.1*xa^2+0.4+comp(2)+p.eta1*habs(2)]);
At=max([0.18+0.6*p.ke,0.4+0.2*comp(1)+0.2*p.eta1,...
    0.8+0.4*comp(2)+0.4*p.eta1]);
Ax=max([0.2*xa+p.ke,0.2*xa+comp(1)+p.eta1,...
    0.2*xa+comp(2)+p.eta1]);
c.Psi=max(1+c.r.*exp(hmax));
c.B=nu+max(hmax+c.r.*exp(hmax)); c.Z=sqrt(2*c.B);
c.L1=At+Ax*(0.1*xa^2+c.K1+c.Z+c.mu10);
c.ell2=p.eta2-p.eta1*c.Psi/2;
if c.ell2>0
    c.D_R=c.mu10+p.tau*c.L1+((c.Psi+1)+c.m2)^2/(4*c.ell2);
else
    c.D_R=Inf;
end
c.filter_limit=p.tau*c.L1;
c.parameter_checks=R>c.mu10 && c.whole_box_gap>=0 && ...
    c.sharing_gap>=0 && c.ell2>0 && c.D_R<=R;
assert(c.parameter_checks,'HCF scalar sufficient certificate failed.');
z0=cell(3,1); c.initial=zeros(3,5);
for ci=1:3
    z=zeros(4,1); [~,o]=HCF_revised(0,z,p,ci); z(3)=o.theta;
    [~,o]=HCF_revised(0,z,p,ci); E=0.5*(z(2)-z(3))^2;
    H=[o.h1,o.h2]+c.r.*exp([o.h1,o.h2])-E;
    good=min(H)>=0 && abs(o.dhat2)<=c.P2 && abs(z(3))<=c.K1 ...
        && abs(z(3)-o.theta)<=c.filter_limit;
    assert(good,'HCF initialization failed for case %d.',ci);
    z0{ci}=z; c.initial(ci,:)=[H,o.theta,o.u,E];
end
end
