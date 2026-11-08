function plot_nine_scenarios(results_dir)
% Read HCF/RCBF/ACBF trajectories for three disturbances and create six figures.
% The input folder must contain METHOD_caseN.csv for all nine combinations.

if nargin < 1 || isempty(results_dir)
    results_dir = resolve_results_dir(fileparts(mfilename('fullpath')));
end
assert(exist(results_dir, 'dir') == 7, 'Results folder does not exist: %s', results_dir);

methods = {'HCF', 'RCBF', 'ACBF'};
method_labels = {'HCF', 'RCBF', 'ACBF'};
scenario_names = {'Persistent periodic', 'Smooth pulse', 'Slow bias + periodic'};
colors = [0.00, 0.00, 1.00; 0.72, 0.72, 0.00; 0.90, 0.55, 0.00];
styles = {'-', '--', '-.'};
needed = {'t', 'x1', 'x2', 'h1', 'h2', 'u', 'yd'};

trajectories = cell(3, 3);
for ci = 1:3
    for mi = 1:3
        file_path = fullfile(results_dir, sprintf('%s_case%d.csv', methods{mi}, ci));
        assert(exist(file_path, 'file') == 2, 'Missing trajectory file: %s', file_path);
        opts = detectImportOptions(file_path);
        opts.Delimiter = ',';
        opts.SelectedVariableNames = needed;
        opts.VariableNamingRule = 'preserve';
        trajectories{mi, ci} = readtable(file_path, opts);
    end
end

for ci = 1:3
    fig = figure('Visible', 'on', 'Color', 'w', 'Position', [150 + 30*(ci-1), 100 + 24*(ci-1), 1050, 650], ...
        'Name', sprintf('Tracking comparison - Disturbance %d', ci), 'NumberTitle', 'off');
    movegui(fig, 'center');
    hold on;
    for mi = 1:3
        T = trajectories{mi, ci};
        plot(T.t, T.x1, 'Color', colors(mi, :), 'LineStyle', styles{mi}, ...
            'LineWidth', 1.5, 'DisplayName', method_labels{mi});
    end
    T = trajectories{1, ci};
    y1 = T.x1 - T.h1;
    y2 = T.x1 + T.h2;
    plot(T.t, T.yd, 'r--', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot(T.t, y1, ':', 'Color', [0.10, 0.35, 0.30], ...
        'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(T.t, y2, ':', 'Color', [0.10, 0.35, 0.30], ...
        'LineWidth', 1.6, 'HandleVisibility', 'off');
    [~, i1] = min(abs(T.t - 5));
    [~, i2] = min(abs(T.t - 9));
    [~, id] = min(abs(T.t - 7));
    hold off;
    xlim([0, 50]);
    xticks(0:10:50);
    ylim([-3, 3]);
    yticks(-3:1.5:3);
    grid on;
    box on;
    xlabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('$y$', 'Interpreter', 'latex', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    title(sprintf('Tracking comparison - Disturbance %d (%s)', ci, scenario_names{ci}), ...
        'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Orientation', 'horizontal', 'Location', 'north', ...
        'FontName', 'Times New Roman', 'FontSize', 10);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, ...
        'LineWidth', 0.8, 'TickDir', 'in');
    add_label_arrow(gca, T.t(i1) - 1.1, y1(i1) - 0.42, ...
        T.t(i1), y1(i1), '$y_1$', [0.10, 0.35, 0.30]);
    add_label_arrow(gca, T.t(i2) + 1.3, y2(i2) + 0.48, ...
        T.t(i2), y2(i2), '$y_2$', [0.10, 0.35, 0.30]);
    add_label_arrow(gca, T.t(id) + 0.8, T.yd(id) + 0.55, ...
        T.t(id), T.yd(id), '$y_d$', [0.85, 0.00, 0.00]);
end

for ci = 1:3
    fig = figure('Visible', 'on', 'Color', 'w', 'Position', [190 + 30*(ci-1), 90 + 24*(ci-1), 950, 700], ...
        'Name', sprintf('Safety margin and control input - Disturbance %d', ci), 'NumberTitle', 'off');
    movegui(fig, 'center');
    ax1 = subplot(2, 1, 1);
    hold on;
    for mi = 1:3
        T = trajectories{mi, ci};
        plot(T.t, min(T.h1, T.h2), 'Color', colors(mi, :), ...
            'LineStyle', styles{mi}, 'LineWidth', 1.5, ...
            'DisplayName', method_labels{mi});
    end
    yline(0, 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    hold off;
    xlim([0, 50]);
    if ci == 3
        xticks(0:5:50);
    else
        xticks(0:10:50);
    end
    grid on;
    box on;
    ylabel('$h_m(t)$', 'Interpreter', 'latex', ...
        'FontName', 'Times New Roman', 'FontSize', 12);
    legend('Location', 'northeast', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, ...
        'LineWidth', 0.8, 'TickDir', 'in');

    ax2 = subplot(2, 1, 2);
    hold on;
    for mi = 1:3
        T = trajectories{mi, ci};
        plot(T.t, T.u, 'Color', colors(mi, :), 'LineStyle', styles{mi}, ...
            'LineWidth', 1.5, 'DisplayName', method_labels{mi});
    end
    T = trajectories{1, ci};
    hold off;
    xlim([0, 50]);
    if ci == 3
        xticks(0:5:50);
    else
        xticks(0:10:50);
    end
    u_all = cell(1, 3);
    for mi = 1:3
        u_all{mi} = trajectories{mi, ci}.u;
    end
    umin = min(cellfun(@min, u_all));
    umax = max(cellfun(@max, u_all));
    upad = max(0.08 * (umax - umin), 0.5);
    ylim([umin - upad, umax + upad]);
    grid on;
    box on;
    xlabel('Time(s)', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel('u', 'FontName', 'Times New Roman', 'FontSize', 12);
    legend('Location', 'northeast', 'FontName', 'Times New Roman', 'FontSize', 10);
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, ...
        'LineWidth', 0.8, 'TickDir', 'in');
    linkaxes([ax1, ax2], 'x');
    sgtitle(sprintf('Case %d', ci), 'FontName', 'Times New Roman', ...
        'FontSize', 14, 'FontWeight', 'bold');

end

fprintf('Six figures are open in MATLAB. Save or edit them as needed.\n');
end

function results_dir = resolve_results_dir(start_dir)
candidates = dir(fullfile(start_dir, 'results_*'));
valid = false(size(candidates));
for i = 1:numel(candidates)
    if ~candidates(i).isdir
        continue;
    end
    d = fullfile(start_dir, candidates(i).name);
    valid(i) = exist(fullfile(d, 'HCF_case1.csv'), 'file') == 2 && ...
        exist(fullfile(d, 'RCBF_case3.csv'), 'file') == 2 && ...
        exist(fullfile(d, 'ACBF_case3.csv'), 'file') == 2;
end
candidates = candidates(valid);
assert(~isempty(candidates), 'No complete results_* folder was found under %s.', start_dir);
[~, order] = sort([candidates.datenum], 'descend');
results_dir = fullfile(start_dir, candidates(order(1)).name);
end

function add_label_arrow(ax, label_x, label_y, target_x, target_y, label, color)
ax_pos = get(ax, 'Position');
x_limits = get(ax, 'XLim');
y_limits = get(ax, 'YLim');
label_nx = ax_pos(1) + ax_pos(3) * (label_x - x_limits(1)) / diff(x_limits);
label_ny = ax_pos(2) + ax_pos(4) * (label_y - y_limits(1)) / diff(y_limits);
target_nx = ax_pos(1) + ax_pos(3) * (target_x - x_limits(1)) / diff(x_limits);
target_ny = ax_pos(2) + ax_pos(4) * (target_y - y_limits(1)) / diff(y_limits);
annotation(gcf, 'textarrow', [label_nx, target_nx], [label_ny, target_ny], ...
    'String', label, 'Interpreter', 'latex', 'Color', color, ...
    'FontName', 'Times New Roman', 'FontSize', 12, ...
    'HeadStyle', 'vback2', 'HeadLength', 5, 'HeadWidth', 5, ...
    'LineWidth', 0.8);
end

