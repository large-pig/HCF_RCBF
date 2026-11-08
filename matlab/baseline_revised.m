function [dz,o] = baseline_revised(t,z,p,case_id,method,factor)
% Engineering baseline adaptations: see README for exact scope.
s=study_signals(t,case_id); x=z(1); v=z(2); f1=0.1*x^2;
f2=0.1*x*v-0.2*x; g2=1+x^2;
dh1=z(3)+p.c1*x; dh2=z(4)+p.c2*v;
dh1dot=p.d1filter*(dh1-z(5)); xdot=f1+v+dh1;
alpha=-f1+s.dyd-p.ke*(x-s.yd)-dh1;
alphadot=-0.2*x*xdot+s.ddyd-p.ke*(xdot-s.dyd)-dh1dot;
unom=(-f2+alphadot-p.k2*(v-alpha)-dh2)/g2;
h1=x-s.y1; h2=s.y2-x; k1=p.hocbf1; k2=p.hocbf2;
if strcmp(method,'ACBF')
    psi1=xdot-s.dy1+k1*h1; psi2=s.dy2-xdot+k1*h2;
    common=0.2*x*xdot+f2+dh2+dh1dot;
    a1=common-s.ddy1+k1*(xdot-s.dy1)+k2*psi1;
    a2=s.ddy2-common+k1*(s.dy2-xdot)+k2*psi2;
    m1=max(0,z(6)); m2=max(0,z(7));
    th1=p.adapt_gain*exp(-p.adapt_slope*max(psi1,0))-p.adapt_decay*z(6);
    th2=p.adapt_gain*exp(-p.adapt_slope*max(psi2,0))-p.adapt_decay*z(7);
elseif strcmp(method,'RCBF')
    xd0=f1+v; psi1=xd0-s.dy1+k1*h1; psi2=s.dy2-xd0+k1*h2;
    common=0.2*x*xd0+f2;
    a1=common-s.ddy1+k1*(xd0-s.dy1)+k2*psi1;
    a2=s.ddy2-common+k1*(s.dy2-xd0)+k2*psi2;
    margin=factor*(abs(0.2*x+k1+k2)*s.bound(1)+s.bound(2)+s.bound(3));
    m1=margin; m2=margin; th1=0; th2=0;
else
    error('method must be ACBF or RCBF');
end
lo=(-a1+m1)/g2; hi=(a2-m2)/g2;
if lo<=hi
    u=min(hi,max(lo,unom)); gap=0;
else
    u=(lo+hi)/2; gap=lo-hi; % minimax fallback, not a feasible QP
end
dz=[f1+v+s.d1;f2+g2*u+s.d2;-p.c1*(f1+v+dh1);...
    -p.c2*(f2+g2*u+dh2);p.d1filter*(dh1-z(5));th1;th2];
o=struct('u',u,'h1',h1,'h2',h2,'yd',s.yd,'d1',s.d1,'d2',s.d2,...
    'dhat2',dh2,'qp_gap',gap,'lower_residual',a1+g2*u-m1,...
    'upper_residual',a2-g2*u-m2,'theta1',z(6),'theta2',z(7),...
    'nominal_u',unom,'lower_u',lo,'upper_u',hi);
end
