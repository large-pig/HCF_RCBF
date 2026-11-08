function s = study_signals(t,case_id)
% Common reference/boundaries and three smooth bounded disturbance profiles.
s.y1=-1.1+0.1*sin(2*t); s.y2=1.1-0.2*sin(2*t);
s.dy1=0.2*cos(2*t); s.dy2=-0.4*cos(2*t);
s.ddy1=-0.4*sin(2*t); s.ddy2=0.8*sin(2*t);
s.yd=2*sin(0.3*t); s.dyd=0.6*cos(0.3*t); s.ddyd=-0.18*sin(0.3*t);
switch case_id
    case 1
        s.d1=0.3*sin(t)+0.3*cos(0.5*t);
        s.d2=0.3*sin(0.5*t)+0.3*cos(t);
        s.bound=[0.6,0.45,0.6];
        s.name='Periodic';
    case 2
        s.d1=0.8*exp(-((t-20)/1.5)^2);
        s.d2=1.2*exp(-((t-20.5)/1.5)^2);
        s.bound=[0.8,0.8*sqrt(2/exp(1))/1.5,1.2];
        s.name='Smooth pulse';
    case 3
        s.d1=0.5*(1-exp(-t/15))+0.2*sin(0.7*t);
        s.d2=0.8*(1-exp(-t/15))+0.4*sin(0.5*t);
        s.bound=[0.7,0.5/15+0.14,1.2];
        s.name='Slow bias + periodic';
    otherwise
        error('case_id must be 1, 2, or 3.');
end
% s.bound = [bound on |d1|, bound on |dot d1|, bound on |d2|].
end
