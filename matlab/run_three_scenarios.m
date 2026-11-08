function [metrics,outdir] = run_three_scenarios(outdir)
% Run in MATLAB: [metrics,outdir] = run_three_scenarios;
% Requires Optimization Toolbox for Xiao PACBF quadprog.
% Continuous feedback: recomputed on each ODE RHS call, no ZOH or saturation.
folder=fileparts(mfilename('fullpath')); addpath(folder);
assert(exist('quadprog','file')~=0,'ACBF requires Optimization Toolbox (quadprog).');
if nargin<1
    outdir=fullfile(folder,['results_',datestr(now,'yyyymmdd_HHMMSS')]);
end
assert(~exist(outdir,'dir'),'Output directory already exists; choose a new one.');
mkdir(outdir); params=study_parameters();
[cert,initialHCF]=hcf_certificate(params.HCF,params.R,params.nu);
save(fullfile(outdir,'parameters_certificate.mat'),'params','cert','initialHCF');
write_json(fullfile(outdir,'parameters.json'),params);
write_json(fullfile(outdir,'certificate.json'),cert);
methods={'HCF','RCBF','ACBF'}; blocks=cell(3,3); summaries=cell(9,1); entry=0;
times=unique([(0:0.00005:0.2)';(0.2025:0.0025:params.duration)';params.duration]);
for mi=1:3
    method=methods{mi}; p=params.(method);
    for ci=1:3
        fprintf('%s, scenario %d\n',method,ci);
        if strcmp(method,'HCF')
            n=4; physical0=initialHCF{ci};
        elseif strcmp(method,'RCBF')
            n=7; physical0=zeros(n,1);
        else
            n=7; physical0=[zeros(5,1);p.pacbf_target;p.pacbf_target];
        end
        z0=[physical0;zeros(3,1)];
        rhs=@(t,z) augmented(t,z,n,method,p,ci);
        opts=odeset('RelTol',2e-9,'AbsTol',2e-11,'MaxStep',0.0025);
        tic;
        if strcmp(method,'HCF')
            [t,z]=ode15s(rhs,times,z0,opts);
        else
            [t,z]=ode45(rhs,times,z0,opts);
        end
        wall=toc;
        assert(numel(t)==numel(times),'Incomplete trajectory for %s case %d.',method,ci);
        values=cell(numel(t),1);
        for k=1:numel(t)
            [~,values{k}]=controller(t(k),z(k,1:n)',method,p,ci);
        end
        T=table(t,z(:,1),z(:,2),'VariableNames',{'t','x1','x2'});
        for k=1:n, T.(sprintf('state%d',k))=z(:,k); end
        fields=fieldnames(values{1});
        for k=1:numel(fields)
            T.(fields{k})=cellfun(@(v)v.(fields{k}),values);
        end
        if strcmp(method,'HCF')
            E=0.5*(z(:,2)-z(:,3)).^2;
            T.H1=T.h1+cert.r(1)*exp(T.h1)-E;
            T.H2=T.h2+cert.r(2)*exp(T.h2)-E;
            T.filter_error=z(:,3)-T.theta;
        end
        writetable(T,fullfile(outdir,sprintf('%s_case%d.csv',method,ci)));
        blocks{mi,ci}=T;
        timing=zeros(100,1); indices=round(linspace(1,numel(t),100));
        for k=1:100
            i=indices(k); tic; controller(t(i),z(i,1:n)',method,p,ci);
            timing(k)=toc*1e6;
        end
        duration=t(end)-t(1);
        q=struct('method',method,'case_id',ci,'duration',duration,...
            'rmse',sqrt(z(end,n+1)/duration),'L2',sqrt(z(end,n+1)),...
            'u_rms',sqrt(z(end,n+2)/duration),'u_peak_sampled',max(abs(T.u)),...
            'u2_integral',z(end,n+2),'observer_rmse',sqrt(z(end,n+3)/duration),...
            'min_h1',min(T.h1),'min_h2',min(T.h2),...
            'max_violation',max([0;-T.h1;-T.h2]),'qp_max_residual',max(T.qp_gap),...
            'composite_H1_min',NaN,'composite_H2_min',NaN,...
            'max_filter_error',NaN,'screen_passed',false,...
            'controller_rhs_us_median',median(timing),'solver_wall_s',wall);
        q.screen_passed=q.max_violation<=1e-6 && q.qp_max_residual<=1e-6;
        if strcmp(method,'HCF')
            q.composite_H1_min=min(T.H1);q.composite_H2_min=min(T.H2);
            q.max_filter_error=max(abs(T.filter_error));
            q.screen_passed=q.screen_passed && min([T.H1;T.H2])>=-1e-6 ...
                && q.max_filter_error<=cert.filter_limit+1e-6;
        end
        entry=entry+1;summaries{entry}=q;
    end
end
metrics=struct2table(vertcat(summaries{:}));
writetable(metrics,fullfile(outdir,'metrics.csv'));
save(fullfile(outdir,'all_results.mat'),'metrics','blocks','params','cert');
plot_comparison(blocks,outdir);
disp(metrics(:,{'method','case_id','rmse','u_rms','u_peak_sampled','max_violation','screen_passed'}));
fprintf('Results: %s\n',outdir);
fprintf('Extrema are over the output grid; integral metrics use ODE states.\n');
fprintf('MATLAB outputs must be compared with Python references; superiority is not hardcoded.\n');
end

function [dz,o]=controller(t,z,method,p,ci)
switch method
    case 'HCF', [dz,o]=HCF_revised(t,z,p,ci);
    case 'RCBF', [dz,o]=baseline_revised(t,z,p,ci,'RCBF',1);
    case 'ACBF', [dz,o]=Xiao_PACBF_full(t,z,p,ci);
    otherwise, error('Unknown method.');
end
end

function dz=augmented(t,z,n,method,p,ci)
[d,o]=controller(t,z(1:n),method,p,ci);
dz=[d;(z(1)-o.yd)^2;o.u^2;(o.d2-o.dhat2)^2];
end

function write_json(path,value)
f=fopen(path,'w','n','UTF-8'); assert(f>=0,'Cannot open output file.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s',jsonencode(value));
end

function plot_comparison(blocks,outdir)
names={'HCF','RCBF-HOCBF','Xiao PACBF robust'};
fig=figure('Visible','off','Position',[100,100,1300,1000]);
for ci=1:3
    for col=1:3
        subplot(3,3,(ci-1)*3+col);hold on;
        for mi=1:3
            T=blocks{mi,ci};
            if col==1,v=T.x1;elseif col==2,v=min(T.h1,T.h2);else,v=T.u;end
            plot(T.t,v,'LineWidth',1,'DisplayName',names{mi});
        end
        if col==1
            t=blocks{1,ci}.t;
            plot(t,2*sin(.3*t),'k:','DisplayName','reference');
            plot(t,-1.1+.1*sin(2*t),'k--','HandleVisibility','off');
            plot(t,1.1-.2*sin(2*t),'k--','HandleVisibility','off');
            ylabel('output');
        elseif col==2
            yline(0,'k--','HandleVisibility','off');ylabel('min(h1,h2)');
        else,ylabel('input u');end
        title(sprintf('Scenario %d',ci));xlabel('Time (s)');grid on;
        if ci==1 && col==1,legend('Location','best');end
    end
end
saveas(fig,fullfile(outdir,'comparison.png'));
savefig(fig,fullfile(outdir,'comparison.fig'));close(fig);
fig=figure('Visible','off');
for ci=1:3
    subplot(3,1,ci);T=blocks{1,ci};
    plot(T.t,T.H1,T.t,T.H2);hold on;yline(0,'k--');
    title(sprintf('HCF composite barriers, scenario %d',ci));grid on;
    legend('H1','H2');xlabel('Time (s)');
end
saveas(fig,fullfile(outdir,'HCF_composite_barriers.png'));close(fig);
end
