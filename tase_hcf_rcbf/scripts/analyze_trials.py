#!/usr/bin/env python3
"""导出 ROS1 bag 数据并分析实验阶段，不生成或补写不存在的实机数据。"""
import argparse
import csv
import json
import math
import statistics
import tempfile
from pathlib import Path


def number(row, key):
    value = row[key]
    if str(value).lower() in ('true', 'false'):
        return float(str(value).lower() == 'true')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('Nonfinite %s in cycle %s' % (key, row.get('cycle', '?')))
    return result


def trap(rows, function):
    return sum((number(b, 'time_s') - number(a, 'time_s')) *
               (function(a) + function(b)) / 2 for a, b in zip(rows, rows[1:]))


def quantile(values, probability):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(probability * len(ordered)) - 1)]


def metrics(rows):
    if not rows:
        return {'samples': 0}
    t = [number(x, 'time_s') for x in rows]
    if any(b <= a for a, b in zip(t, t[1:])):
        raise ValueError('Controller times must strictly increase; do not concatenate trials.')
    h1 = [number(x, 'q') - number(x, 'lower') for x in rows]
    h2 = [number(x, 'upper') - number(x, 'q') for x in rows]
    result = {'samples': len(rows), 'start_s': t[0], 'end_s': t[-1],
              'observed_duration_s': t[-1] - t[0], 'h1_min_rad': min(h1),
              'h2_min_rad': min(h2), 'max_violation_rad': max(0, -min(h1), -min(h2)),
              'violation_samples': sum(a < 0 or b < 0 for a, b in zip(h1, h2)),
              'fault_samples': sum(number(r, 'fault_reason') != 0 for r in rows)}
    if len(rows) > 1:
        duration = t[-1] - t[0]
        err = lambda r: number(r, 'q') - number(r, 'reference')
        result['tracking_rmse_rad'] = math.sqrt(max(0, trap(rows, lambda r: err(r)**2) / duration))
        result['tracking_l2_rad_sqrt_s'] = math.sqrt(max(0, trap(rows, lambda r: err(r)**2)))
        result['max_record_gap_ms'] = max(b-a for a, b in zip(t, t[1:])) * 1000
        for prefix in ('torque_sent', 'torque_measured', 'torque_robot_desired'):
            fn = lambda r, p=prefix: number(r, p+'_'+str(int(number(r, 'joint_index'))))
            energy = trap(rows, lambda r: fn(r)**2)
            result[prefix+'_rms_nm'] = math.sqrt(max(0, energy / duration))
            result[prefix+'_sq_integral_nm2_s'] = energy
            result[prefix+'_peak_sampled_nm'] = max(abs(fn(r)) for r in rows)
    for field in ('top_filter_active', 'safety_filter_active', 'nominal_outside_actuator',
                  'actuator_projection_changed', 'actuator_constraint_active', 'post_clip_changed'):
        result[field+'_samples'] = sum(number(r, field) >= .5 for r in rows)
    for field in ('control_compute_us', 'previous_update_total_us'):
        timed = [number(r, field) for r in rows if field != 'previous_update_total_us' or number(r, 'previous_timed_cycle') > 0]
        if timed:
            result[field] = {'mean': statistics.mean(timed), 'p99': quantile(timed, .99), 'max': max(timed)}
    return result


def analyze_one(filename):
    with Path(filename).open(encoding='utf-8-sig', newline='') as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError('Empty CSV: '+str(filename))
    if any(number(r, 'schema_version') != 2 for r in rows):
        raise ValueError('Only the named schema-2 messages are supported by this script.')
    cycles = [number(r, 'cycle') for r in rows]
    if any(b <= a for a, b in zip(cycles, cycles[1:])):
        raise ValueError('Cycle IDs must strictly increase.')
    for key in ('mode', 'joint_index', 'run_duration_s'):
        if len({number(r, key) for r in rows}) != 1:
            raise ValueError('Mixed trial metadata: '+key)
    duration = number(rows[0], 'run_duration_s')
    planned = [r for r in rows if number(r, 'time_s') < duration]
    active = [r for r in rows if number(r, 'phase') == 1]
    result = {'source': str(filename), 'method': 'HCF' if number(rows[0], 'mode') == 0 else 'RCBF',
              'trial_id': rows[0].get('trial_id', Path(filename).stem),
              'scenario': rows[0].get('scenario', 'unknown'),
              'first_cycle': cycles[0], 'startup_cycles_not_recorded': cycles[0]-1,
              'unrecorded_cycle_ids_between_records': sum(b-a-1 for a, b in zip(cycles, cycles[1:])),
              'dropped_log_total_last': number(rows[-1], 'dropped_log_total'),
              'normal_finish_observed': any(number(r, 'phase') == 2 for r in rows),
              'any_fault': any(number(r, 'fault_reason') != 0 for r in rows),
              'all_recorded': metrics(rows),
              # 统计窗口包含故障后的位置保持样本，不得只提取故障前的正常区间，
              # 以免遗漏失败实验并造成性能指标偏差。
              'planned_window_including_faults': metrics(planned),
              'active_control_only': metrics(active),
              'phases': {str(p): metrics([r for r in rows if number(r, 'phase') == p]) for p in (0, 1, 2, 3)}}
    result['window_average_residuals_NOT_certified_bounds'] = []
    for sec in range(1, int(duration)-1):
        segment = [r for r in active if sec <= number(r, 'time_s') < sec+1]
        if len(segment) < 2:
            continue
        dt = number(segment[-1], 'time_s')-number(segment[0], 'time_s')
        residual = (number(segment[-1], 'dq_physical')-number(segment[0], 'dq_physical')-
                    trap(segment, lambda r: number(r, 'command_acceleration'))) / dt
        result['window_average_residuals_NOT_certified_bounds'].append({'start_s': sec, 'mean_rad_s2': residual})
    return result


def flatten(message, prefix='', result=None):
    if result is None:
        result = {}
    if hasattr(message, '__slots__'):
        for key in message.__slots__:
            flatten(getattr(message, key), (prefix+'_' if prefix else '')+key, result)
    elif isinstance(message, (tuple, list)):
        for i, item in enumerate(message):
            flatten(item, prefix+'_'+str(i), result)
    else:
        result[prefix] = message
    return result


def export_bag(args):
    try:
        import rosbag
    except ImportError as error:
        raise ValueError('Run bag export in ROS1 with rosbag Python available.') from error
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    config = []
    count = 0
    schema = None
    with rosbag.Bag(args.input, 'r') as bag, output.open('w', encoding='utf-8-sig', newline='') as file:
        writer = None
        for topic, msg, stamp in bag.read_messages(topics=[args.topic, args.config_topic]):
            if topic == args.config_topic:
                config.append({'bag_time': '%d.%09d' % (stamp.secs, stamp.nsecs), 'raw': msg.data})
                continue
            row = flatten(msg)
            row['bag_time_s'] = '%d.%09d' % (stamp.secs, stamp.nsecs)
            if writer is None:
                columns = list(row)
                writer = csv.DictWriter(file, fieldnames=columns)
                writer.writeheader()
                schema = {
                    'topic': args.topic,
                    'message_type': getattr(msg, '_type', ''),
                    'md5sum': getattr(msg, '_md5sum', ''),
                    'message_definition': getattr(msg, '_full_text', ''),
                    'columns': columns,
                }
            writer.writerow(row)
            count += 1
    output.with_suffix('.configuration.json').write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    if count == 0:
        raise ValueError('No sample messages found; check topic and bag.')
    schema['rows'] = count
    output.with_suffix('.schema.json').write_text(
        json.dumps(schema, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print('Exported %d recorded samples to %s' % (count, output))


def export_topic(input_path, output_path, topic_name):
    """将一个 ROS 消息话题展开为 CSV，同时导出消息字段结构。"""
    try:
        import rosbag
    except ImportError as error:
        raise ValueError('Run bag export in ROS1 with rosbag Python available.') from error

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    schema = None

    with rosbag.Bag(str(input_path), 'r') as bag, output.open(
        'w', encoding='utf-8-sig', newline=''
    ) as file:
        writer = None
        for _, message, stamp in bag.read_messages(topics=[topic_name]):
            row = flatten(message)
            row['bag_time_s'] = '%d.%09d' % (stamp.secs, stamp.nsecs)
            if writer is None:
                columns = list(row)
                writer = csv.DictWriter(file, fieldnames=columns)
                writer.writeheader()
                schema = {
                    'topic': topic_name,
                    'message_type': getattr(message, '_type', ''),
                    'md5sum': getattr(message, '_md5sum', ''),
                    'message_definition': getattr(message, '_full_text', ''),
                    'columns': columns,
                }
            writer.writerow(row)
            count += 1

    if count == 0:
        raise ValueError('No messages found on %s in %s' % (topic_name, input_path))

    schema['rows'] = count
    output.with_suffix('.schema.json').write_text(
        json.dumps(schema, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print('Exported %d messages from %s to %s' % (count, topic_name, output))


def self_test():
    base = dict(schema_version=2, cycle=1, dropped_log_total=0, phase=1, mode=0, joint_index=4,
                time_s=0, run_duration_s=3, q=0, lower=-.5, upper=.5, reference=0,
                dq_physical=0, command_acceleration=0, fault_reason=0, top_filter_active=False,
                safety_filter_active=False, nominal_outside_actuator=False,
                actuator_projection_changed=False, actuator_constraint_active=False, post_clip_changed=False,
                control_compute_us=10, previous_update_total_us=12, previous_timed_cycle=1)
    for prefix in ('torque_sent', 'torque_measured', 'torque_robot_desired'):
        base[prefix+'_4'] = 2
    with tempfile.TemporaryDirectory(prefix='SYNTHETIC_ANALYSIS_TEST_') as folder:
        file = Path(folder)/'synthetic.csv'
        with file.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(base))
            writer.writeheader()
            for cycle, (t, q, phase) in enumerate([(0, 0, 1), (1, .25, 1), (2, 0, 1), (3, -.6, 2)], 1):
                writer.writerow(dict(base, cycle=cycle, time_s=t, q=q, phase=phase))
        result = analyze_one(file)
        assert result['planned_window_including_faults']['max_violation_rad'] == 0
        assert abs(result['phases']['2']['max_violation_rad']-.1) < 1e-10
        assert abs(result['planned_window_including_faults']['tracking_rmse_rad']-math.sqrt(.03125)) < 1e-10
        assert result['planned_window_including_faults']['torque_sent_sq_integral_nm2_s'] == 8
        assert result['normal_finish_observed'] and not result['any_fault']
        with file.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(base))
            writer.writeheader()
            writer.writerow(base)
            writer.writerow(dict(base, cycle=3, time_s=1, phase=3, fault_reason=3, q=.6))
        failed = analyze_one(file)
        assert failed['any_fault'] and not failed['normal_finish_observed']
        assert failed['planned_window_including_faults']['samples'] == 2
        assert failed['planned_window_including_faults']['max_violation_rad'] > 0
        assert failed['unrecorded_cycle_ids_between_records'] == 1
    print('PASS: synthetic phase separation, time-weighted metrics, CSV flags; no robot data created.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    export = sub.add_parser('export')
    export.add_argument('input')
    export.add_argument('--output', required=True)
    export.add_argument('--topic', default='/tase_comparison/sample')
    export.add_argument('--config-topic', default='/tase_comparison/configuration')
    analyze = sub.add_parser('analyze')
    analyze.add_argument('inputs', nargs='+')
    analyze.add_argument('--output', required=True)
    sub.add_parser('self-test')
    args = parser.parse_args()
    if args.command == 'export':
        export_bag(args)
    elif args.command == 'self-test':
        self_test()
    else:
        results = [analyze_one(name) for name in args.inputs]
        output = Path(args.output)
        output.mkdir(parents=True, exist_ok=True)
        (output/'trial_details.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
        table = []
        for r in results:
            values = {k: v for k, v in r['planned_window_including_faults'].items() if isinstance(v, (int, float))}
            table.append(dict(source=r['source'], method=r['method'], scenario=r['scenario'], trial_id=r['trial_id'], normal_finish_observed=r['normal_finish_observed'], any_fault=r['any_fault'], **values))
        with (output/'planned_window_metrics.csv').open('w', encoding='utf-8-sig', newline='') as file:
            columns = sorted({k for row in table for k in row})
            writer = csv.DictWriter(file, fieldnames=columns)
            writer.writeheader()
            writer.writerows(table)
        print('Analyzed %d independent files; failed/holding phases remain reported.' % len(results))


if __name__ == '__main__':
    main()
