function selftest_three_scenarios()
% Run before the full simulation. Requires Optimization Toolbox.
p=study_parameters();[c,z]=hcf_certificate(p.HCF,p.R,p.nu);
assert(abs(c.D_R-1.4909803921568627)<1e-9,'Certificate differs from Python reference.');
assert(abs(c.L1-865.64042895758803)<1e-8,'Derivative bound differs.');
assert(abs(c.filter_limit-0.040000000000000001)<1e-12);
for ci=1:3
    [d,o]=HCF_revised(0,z{ci},p.HCF,ci);
    assert(abs(o.u-17.697282280999296)<1e-9);
    assert(numel(d)==4 && all(isfinite(d)) && o.qp_gap==0);
    [~,r]=baseline_revised(0,zeros(7,1),p.RCBF,ci,'RCBF',1);
    assert(abs(r.u-20)<1e-9 && r.qp_gap==0);
    [~,a]=Xiao_PACBF_full(0,[zeros(5,1);10;10],p.ACBF,ci);
    assert(abs(a.u-20)<1e-4 && a.qp_gap<1e-6);
    for t=[0,3,19,21,40]
        s=study_signals(t,ci);
        assert(abs(s.d1)<=s.bound(1)+1e-12 && abs(s.d2)<=s.bound(3)+1e-12);
        assert(s.y2-s.y1>=1.9-1e-12);
        if t>0
            h=1e-4;sp=study_signals(t+h,ci);sm=study_signals(t-h,ci);
            assert(abs((sp.d1-sm.d1)/(2*h))<=s.bound(2)+1e-6);
            assert(abs((sp.y1-sm.y1)/(2*h)-s.dy1)<1e-6);
            assert(abs((sp.y2-sm.y2)/(2*h)-s.dy2)<1e-6);
        end
    end
end
fprintf('Certificate, initial control, QP feasibility and signal checks passed.\n');
fprintf('Sample checks supplement, but do not replace, analytic envelope derivations.\n');
end
