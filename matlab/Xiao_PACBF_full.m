function [dz,out] = Xiao_PACBF_full(t,z,p,case_id)
% Xiao et al., TAC 2022, parameter adaptive CBF (PACBF), relative degree two.
% z=[x1;x2;observer_p1;observer_p2;filter_dhat1;penalty_lower;penalty_upper].
% The two safety constraints each receive one auxiliary penalty state a_j,
% with dynamics a_j_dot = nu_j. The second PACBF penalty is fixed to p.hocbf2.
% QP decision vector w=[u;nu_lower;nu_upper;delta_lower;delta_upper].
% Requires Optimization Toolbox (quadprog). The nominal Xiao PACBF is
% robustly extended using bounds on |d1| and |d2|; no d1 derivative is used.
if ~isfield(p,'pacbf_target'), p.pacbf_target=10; end
if ~isfield(p,'pacbf_positive_gain'), p.pacbf_positive_gain=5; end
if ~isfield(p,'pacbf_clf_rate'), p.pacbf_clf_rate=2; end
if ~isfield(p,'pacbf_nu_weight'), p.pacbf_nu_weight=1; end
if ~isfield(p,'pacbf_slack_weight'), p.pacbf_slack_weight=100; end
if ~isfield(p,'pacbf_dual_tol'), p.pacbf_dual_tol=1e-7; end
if ~isfield(p,'pacbf_bound_factor'), p.pacbf_bound_factor=1; end
s=study_signals(t,case_id); x=z(1); v=z(2); ap=z(6:7);
f1=.1*x^2; f2=.1*x*v-.2*x; g2=1+x^2;
dhat1=z(3)+p.c1*x; dhat2=z(4)+p.c2*v;
dhat1dot_est=p.d1filter*(dhat1-z(5));
xdot_est=f1+v+dhat1;
alpha=-f1+s.dyd-p.ke*(x-s.yd)-dhat1;
alphadot=-.2*x*xdot_est+s.ddyd-p.ke*(xdot_est-s.dyd)-dhat1dot_est;
unom=(-f2+alphadot-p.k2*(v-alpha)-dhat2)/g2;
h=[x-s.y1;s.y2-x];
% Robust first-level barriers for matched and unmatched disturbances:
% psi_j=s_j*(f1+x2-bound_dot)-mu1+a_j*h_j.
% hdot_j+a_j*h_j >= psi_j. A bound on dot(d1) is not required.
mu1=p.pacbf_bound_factor*s.bound(1); mu2=p.pacbf_bound_factor*s.bound(3);
hdot_nom=[f1+z(2)-s.dy1;s.dy2-f1-z(2)];
drift=.2*x*(f1+z(2))+f2;
hddot0=[drift-s.ddy1;s.ddy2-drift];
psi1=hdot_nom-mu1+ap.*h;
margin=abs(.2*x+ap)*mu1+mu2;
base=hddot0+ap.*hdot_nom+p.hocbf2*psi1-margin;
% -[s_j*g2, h_j, 0]*w <= base_j (j=1,2).
% -nu_j <= kappa*a_j (CBF for a_j >=0).
% 2(a_j-a*)nu_j -delta_j <= -eps(a_j-a*)^2 (soft CLF).
v=ap-p.pacbf_target;
A=[-g2,-h(1),0,0,0;g2,0,-h(2),0,0;
   0,-1,0,0,0;0,0,-1,0,0;
   0,2*v(1),0,-1,0;0,0,2*v(2),0,-1];
b=[base(1);base(2);p.pacbf_positive_gain*ap(1);...
   p.pacbf_positive_gain*ap(2);-p.pacbf_clf_rate*v(1)^2;...
   -p.pacbf_clf_rate*v(2)^2];
% As in Xiao's PACBF objective: quadratic tracking/CLF-slack penalties and
% linear weights on the auxiliary controls; second penalty is fixed here.
H=diag([1,0,0,p.pacbf_slack_weight,p.pacbf_slack_weight]);
f=[-unom;p.pacbf_nu_weight;p.pacbf_nu_weight;0;0];
options=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex',...
                     'ConstraintTolerance',p.pacbf_dual_tol);
[w,~,flag,info]=quadprog(H,f,A,b,[],[],[],[],[],options);
if flag<=0 || isempty(w)
    error('Xiao_PACBF:QPInfeasible','PACBF QP failed at t=%.9g, flag=%d: %s',t,flag,info.message);
end
u=w(1); nu=w(2:3); slack=w(4:5);
dz=[f1+z(2)+s.d1; f2+g2*u+s.d2;...
    -p.c1*(f1+z(2)+dhat1); -p.c2*(f2+g2*u+dhat2);...
    p.d1filter*(dhat1-z(5)); nu];
out=struct('u',u,'h1',h(1),'h2',h(2),'yd',s.yd,'d1',s.d1,'d2',s.d2,...
    'dhat2',dhat2,'qp_gap',max([0;A*w-b]),'psi1_lower',psi1(1),...
    'psi1_upper',psi1(2),'penalty_lower',ap(1),'penalty_upper',ap(2),...
    'nu_lower',nu(1),'nu_upper',nu(2),'delta_lower',slack(1),...
    'delta_upper',slack(2),'lower_residual',b(1)-A(1,:)*w,...
    'upper_residual',b(2)-A(2,:)*w,'qp_exitflag',flag,...
    'assumed_mu1',mu1,'assumed_mu2',mu2,'margin_lower',margin(1),'margin_upper',margin(2));
end
