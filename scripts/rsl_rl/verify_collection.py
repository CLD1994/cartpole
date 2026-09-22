"""检查 collect_calibration.py 的输出；只依赖 NumPy，不启动 IsaacLab。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def require(condition, message):
    if not condition:
        raise ValueError(message)


def checksum(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def verify_collection(directory, expected_count=None, check_model=False):
    root = Path(directory).resolve()
    require(not (root / 'failure.json').exists(), '存在 failure.json，请先查看失败原因')
    meta = json.loads((root / 'metadata.json').read_text(encoding='utf-8'))
    require(meta['collection_version'] == 1 and meta['collection_status'] == 'complete', '采集未成功完成或版本不支持')
    for name, record in {**meta['files'], **meta['configuration_files']}.items():
        require(Path(name).name == name, f'无效文件名：{name}')
        require(checksum(root / name) == record['sha256'], f'SHA-256 不一致：{name}')
    require(meta['checkpoint'] == meta['model']['path'], '模型路径记录不一致')
    if check_model:
        require(checksum(Path(meta['model']['path'])) == meta['model']['sha256'], '模型 SHA-256 不一致')
    obs = np.load(root / 'candidates.npy', allow_pickle=False)
    count = meta['candidate_count']
    require(obs.shape == (count, 4) and obs.dtype == np.float32 and np.isfinite(obs).all(), '候选观测的形状、类型或有限性错误')
    if expected_count is not None:
        require(count == expected_count, f'期望 {expected_count} 条，实际 {count} 条')
    with np.load(root / 'sample_provenance.npz', allow_pickle=False) as archive:
        trace = {name: archive[name] for name in ('env_id', 'episode_id', 'episode_step', 'policy_step')}
    for name, values in trace.items():
        require(values.shape == (count,) and np.issubdtype(values.dtype, np.integer), f'来源列与观测不对齐：{name}')
    env, episode, age, step = (trace[name] for name in ('env_id', 'episode_id', 'episode_step', 'policy_step'))
    require(np.all((env >= 0) & (env < meta['num_envs'])), '环境编号越界')
    require(np.all(episode >= 0) and np.all(step >= meta['warmup_steps']), '回合或时间步非法')
    require(np.all(np.diff(step) >= 0), '时间步不是采集顺序')
    require(np.all(np.where(episode == 0, age == -1, (age >= 0) & (age <= step))), '初始片段或回合步数非法')
    offsets = np.asarray(meta['rollout']['sample_offsets'])
    require(offsets.shape == (meta['num_envs'],), '采样相位维数错误')
    require(np.all(step % meta['sample_stride'] == offsets[env]), '样本与采样相位不一致')
    require(len(np.unique(np.column_stack((env, step)), axis=0)) == count, '同一环境同一时间步重复采样')
    for env_id in np.unique(env):
        selected = env == env_id
        require(np.all(np.diff(episode[selected]) >= 0), '回合编号倒退')
        require(np.all(np.diff(episode[selected]) <= np.diff(step[selected])), '回合增长超过实际时间步')
    identities = np.column_stack((env, episode))
    for env_id, episode_id in np.unique(identities[episode > 0], axis=0):
        selected = (env == env_id) & (episode == episode_id)
        require(len(np.unique(step[selected] - age[selected])) == 1, '同一回合的起始时间不一致')
    report = json.loads((root / 'coverage.json').read_text(encoding='utf-8'))
    require(report['sample_count'] == count, '覆盖报告样本数不一致')
    require(report['unique_observations'] == len(np.unique(obs, axis=0)), '不同观测计数不一致')
    require(report['sampled_episode_count'] == len(np.unique(identities, axis=0)), '回合计数不一致')
    require(report['initial_fragment_count'] == len(np.unique(identities[episode == 0], axis=0)), '初始片段计数不一致')
    columns = dict(zip(meta['observation_order'], obs.T))
    columns['pole_angle_deg'] = np.rad2deg(obs[:, 1].astype(np.float64))
    for name, feature in report['features'].items():
        values = columns[name]
        require(feature['min'] == float(values.min()) and feature['max'] == float(values.max()), f'范围统计错误：{name}')
        masks = [(item, (values >= (-np.inf if item['lower'] is None else item['lower'])) &
                  (values < (np.inf if item['upper'] is None else item['upper']))) for item in feature['bins']]
        masks += [(feature['directions'][direction], mask) for direction, mask in
                  (('negative', values < 0), ('zero', values == 0), ('positive', values > 0))]
        for item, mask in masks:
            require(item['samples'] == int(mask.sum()) and item['episodes'] == len(np.unique(identities[mask], axis=0)), f'分段/方向统计错误：{name}')
        require(sum(item['samples'] for item in feature['bins']) == count, f'分段未覆盖全部数据：{name}')
    if meta['preselection_status'] == 'complete':
        sizes = sorted(int(size) for size in meta['calibration_sets'])
        indices = np.load(root / f'selection_indices_{sizes[-1]}.npy', allow_pickle=False)
        roles = np.load(root / f'selection_roles_{sizes[-1]}.npy', allow_pickle=False)
        require(indices.shape == roles.shape == (sizes[-1],), '初筛索引或角色形状错误')
        require(np.issubdtype(indices.dtype, np.integer) and len(np.unique(indices)) == len(indices)
                and np.all((indices >= 0) & (indices < count)), '初筛索引非法或重复')
        for size in sizes:
            selected = np.load(root / meta['calibration_sets'][str(size)], allow_pickle=False)
            require(np.array_equal(selected, obs[indices[:size]]), f'{size} 条初筛数据未保持原始观测')
            expected = meta['role_counts'][str(size)]
            require(np.isin(roles[:size], [0, 1, 2]).all(), '未知初筛角色')
            require([int((roles[:size] == role).sum()) for role in (0, 1, 2)] ==
                    [expected[name] for name in ('representative', 'core_coverage', 'controlled_tail')], '初筛角色数量错误')
    else:
        require(meta['preselection_status'] == 'not_requested' and not meta['calibration_sets'], '初筛状态错误')
        require(not list(root.glob('calibration_*.npy')), '仅候选模式出现标定文件')
    return {'sample_count': count, 'sampled_episode_count': report['sampled_episode_count'],
            'initial_fragment_count': report['initial_fragment_count'], 'seed': meta['seed'],
            'model_path': meta['model']['path'], 'preselection_status': meta['preselection_status']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--expected-count', type=int)
    parser.add_argument('--check-model', action='store_true', help='在主机核对记录路径下的模型哈希')
    args = parser.parse_args()
    try:
        result = verify_collection(args.directory, args.expected_count, args.check_model)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        print(f'核验失败：{error}', file=sys.stderr)
        return 1
    print('文件与统计核验通过；这不代表量化精度或闭环验收通过。')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
