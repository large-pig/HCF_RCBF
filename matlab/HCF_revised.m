function [dz,o] = HCF_revised(t,z,p,case_id)
% z=[x1;x2;k1;observer_p2]. No disturbance is supplied to the observer.
s=study_signals(t,case_id); x=z(1); v=z(2); k=z(3); dh=z(4)+p.c2*v;
f1=0.1*x^2; f2=0.1*x*v-0.2*x; g2=1+x^2;
h1=x-s.y1; h2=s.y2-x; nominal=-f1+s.dyd-p.ke*(x-s.yd);
a1=f1-s.dy1-exp(-h1)/p.p1+p.eta1*h1;
a2=-f1+s.dy2-exp(-h2)/p.p2+p.eta1*h2;
lambda1=max(0,-(a1+nominal)); lambda2=max(0,-(a2-nominal));
theta=nominal+lambda1-lambda2; C=tanh((lambda1-lambda2)/p.rho);
dk=(theta-k)/p.tau; u=(-f2+C+dk-dh-p.eta2*(v-k))/g2;
dz=[f1+v+s.d1; f2+g2*u+s.d2; dk; -p.c2*(f2+g2*u+dh)];
o=struct('u',u,'h1',h1,'h2',h2,'yd',s.yd,'d1',s.d1,'d2',s.d2,...
    'dhat2',dh,'theta',theta,'k',k,'lambda1',lambda1,'lambda2',lambda2,...
    'C',C,'qp_gap',max(0,-a1-a2),'lower_residual',a1+theta,'upper_residual',a2-theta);
end
