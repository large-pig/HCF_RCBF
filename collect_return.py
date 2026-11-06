#!/usr/bin/env python3
"""所有 rosbag 录制结束后，导出 CSV、统计实验指标并打包原始数据。"""
import argparse
import hashlib
import importlib.util
import json
import os
import platform
import subprocess
import sys
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace


def command(args, timeout=15):
    try:
        out = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, timeout=timeout, check=False)
        return {'command': args, 'returncode': out.returncode, 'output': out.stdout}
    except Exception as error:
        return {'command': args, 'error': str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', help='directory containing the stopped recordings')
    args = parser.parse_args()
    folder = Path(args.folder).expanduser().resolve()
    if not folder.is_dir():
        parser.error('Folder does not exist.')
    if list(folder.glob('*.active')):
        parser.error('An .active bag is present. Stop recording normally and wait; do not rename an unfinished bag.')
    bags = sorted(folder.glob('*.bag'))
    if not bags:
        parser.error('No .bag files found.')
    root = Path(__file__).resolve().parent
    analyzer = root/'tase_hcf_rcbf/scripts/analyze_trials.py'
    spec = importlib.util.spec_from_file_location('tase_analysis', analyzer)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    errors = []
    for bag in bags:
        sample_csv = bag.with_suffix('.csv')
        state_csv = bag.with_name(bag.stem + '_franka_states.csv')
        try:
            module.export_bag(SimpleNamespace(
                input=str(bag),
                output=str(sample_csv),
                topic='/tase_comparison/sample',
                config_topic='/tase_comparison/configuration',
            ))
        except Exception as error:
            errors.append({'file': bag.name, 'topic': '/tase_comparison/sample', 'error': str(error)})
        try:
            module.export_topic(
                str(bag),
                str(state_csv),
                '/franka_state_controller/franka_states',
            )
        except Exception as error:
            errors.append({
                'file': bag.name,
                'topic': '/franka_state_controller/franka_states',
                'error': str(error),
            })
    csvs = [b.with_suffix('.csv') for b in bags if b.with_suffix('.csv').exists()]
    if csvs:
        summary = command([sys.executable, str(analyzer), 'analyze', *map(str, csvs),
                           '--output', str(folder/'analysis')], timeout=180)
    else:
        summary = {'error': 'No CSV exported. Raw bag files are still included.'}
    versions = [command(['rosversion', p]) for p in ('roscpp', 'franka_hw', 'franka_control', 'controller_manager')]
    versions += [command(['pkg-config', '--modversion', 'franka']), command(['uname', '-a'])]
    source_hashes = {}
    for p in root.rglob('*'):
        relative = p.relative_to(root)
        if p.is_file() and not any(x in ('.build', '__pycache__', 'data') for x in relative.parts) and p.suffix not in ('.zip', '.bag', '.csv'):
            source_hashes[relative.as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {'created_local': time.strftime('%Y-%m-%d %H:%M:%S'), 'platform': platform.platform(),
                'python': sys.version, 'ROS_DISTRO': os.environ.get('ROS_DISTRO'), 'versions': versions,
                'export_errors': errors, 'analysis': summary, 'source_sha256': source_hashes,
                'warning': 'Observed data and empirical residuals are not a global disturbance-bound proof.'}
    (folder/'collection_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    output = folder.parent/('TASE_return_'+time.strftime('%Y%m%d_%H%M%S')+'.zip')
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=3, allowZip64=True) as archive:
        for path in folder.rglob('*'):
            if path.is_file():
                archive.write(path, 'data/'+path.relative_to(folder).as_posix())
        for name in source_hashes:
            archive.write(root/name, 'used_source/'+name)
    print('SEND BACK: '+str(output))
    if errors or summary.get('returncode', 0):
        print('Some exports/checks failed. Their errors and raw bags are included; send the ZIP anyway.')


if __name__ == '__main__':
    main()
