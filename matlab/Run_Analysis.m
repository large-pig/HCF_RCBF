% One-click run: self-test, nine MATLAB simulations, then six comparison plots.
clear; clc; close all;

base_dir = fileparts(mfilename('fullpath'));
addpath(base_dir);

% Required before starting: nothing else in this folder.
selftest_three_scenarios;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
results_dir = fullfile(base_dir, ['results_', timestamp]);
[metrics, outdir] = run_three_scenarios(results_dir);
plot_nine_scenarios(outdir);

% Keep the nine requested CSV files and six requested figures together at
% the top level. The complete timestamped run remains in results_dir.
methods = {'HCF', 'RCBF', 'ACBF'};
for ci = 1:3
    for mi = 1:3
        source = fullfile(outdir, sprintf('%s_case%d.csv', methods{mi}, ci));
        copyfile(source, fullfile(base_dir, sprintf('%s_case%d.csv', methods{mi}, ci)));
    end
end
copyfile(fullfile(outdir, 'metrics.csv'), base_dir);

fid = fopen(fullfile(base_dir, 'latest_results_path.txt'), 'w', 'n', 'UTF-8');
assert(fid >= 0, 'Cannot write latest_results_path.txt.');
fprintf(fid, '%s\n', outdir);
fclose(fid);

disp(metrics(:, {'method','case_id','rmse','u_rms','u_peak_sampled', ...
    'max_violation','screen_passed'}));
fprintf('MATLAB rerun complete.\nResults: %s\n', outdir);
