"""训练结束后默认评估已保存动态候选；旧时段提名需显式启用，不修改checkpoint。"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import signal
import time
from pathlib import Path
import re
import statistics
import subprocess
import sys
import uuid


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    tmp.replace(path)


def allowed_checkpoint(path, run):
    """Resolve symlinks before allowing ONLY our two checkpoint layouts."""
    path = Path(path).resolve()
    run = run.resolve()
    regular = path.parent == run and re.fullmatch(r'model_\d+\.pt', path.name)
    candidate = path.parent == run/'training-candidates' and re.fullmatch(r'candidate_\d+\.pt', path.name)
    if not (regular or candidate):
        raise ValueError(f'checkpoint路径不属于本run许可布局: {path}')
    return path


def include_saved_candidates(result, checkpoints):
    """Never discard the training shortlist solely because temporal grouping differs."""
    selected, metadata = result
    included = []
    for checkpoint in checkpoints:
        if Path(checkpoint['path']).name.startswith('candidate_') and checkpoint not in selected:
            selected.append(checkpoint)
            included.append(checkpoint['path'])
    metadata['extra_training_candidates'] = included
    return selected, metadata


def nominate_complete_episodes(run, checkpoints, candidates, window):
    """新run以逐iteration原始计数为权威；缺失整套日志才交给旧日志提名。"""
    episode_file = run/'training-episodes.jsonl'
    checkpoint_file = run/'training-checkpoints.jsonl'
    if not episode_file.exists() and not checkpoint_file.exists():
        return None
    if not episode_file.is_file() or not checkpoint_file.is_file():
        raise ValueError('新回合日志只有一份，不能回退旧标量；请恢复完整文件或显式指定候选')
    rows = [json.loads(line) for line in episode_file.read_text(encoding='utf-8').splitlines() if line.strip()]
    saves = [json.loads(line) for line in checkpoint_file.read_text(encoding='utf-8').splitlines() if line.strip()]
    if not rows or not saves:
        raise ValueError('新回合日志为空，不能用于自动提名')
    def integer(value):
        return isinstance(value,int) and not isinstance(value,bool) and value>=0
    by_iteration = {}
    previous = None
    for row in rows:
        if row.get('schema_version')!=2 or row.get('rank')!=0 or Path(row['run_dir']).resolve()!=run:
            raise ValueError('回合日志schema/rank/run_dir不匹配；请显式指定候选')
        fields = ['iteration','start_iteration','window_start_step','window_end_step','completed','successes','failures']
        if not all(integer(row[k]) for k in fields) or row['successes']+row['failures']!=row['completed']:
            raise ValueError('回合日志计数无效')
        if row['window_end_step']<=row['window_start_step'] or row['iteration'] in by_iteration:
            raise ValueError('回合窗口无效或iteration重复')
        if previous is None:
            if row['iteration']!=row['start_iteration'] or row['window_start_step']!=0:
                raise ValueError('回合日志缺失首个窗口')
        elif (row['iteration']!=previous['iteration']+1 or row['start_iteration']!=previous['start_iteration']
              or row['window_start_step']!=previous['window_end_step']):
            raise ValueError('回合日志窗口不连续，不能混合恢复运行或缺失样本')
        by_iteration[row['iteration']]=row
        previous=row
    bindings = {}
    for record in saves:
        if record.get('schema_version')!=1:
            raise ValueError('未知checkpoint日志schema')
        path = str(Path(record['checkpoint']).resolve())
        iteration = record['saved_after_update_iteration']
        if not integer(iteration) or iteration not in by_iteration or record['training_iteration_metrics']!=by_iteration[iteration]:
            raise ValueError('checkpoint保存记录与iteration计数不一致')
        allowed_checkpoint(path, run)  # Includes valid historical/evicted candidate paths.
        if int(Path(path).stem.rsplit('_', 1)[1]) != iteration:
            raise ValueError('保存记录路径轮次与iteration不一致')
        bindings[path]=iteration  # 最终保存可重复同一路径，按生产者约定取最后一条。
    mapped=[]
    for candidate in checkpoints:
        row=dict(candidate)
        if candidate['path'] not in bindings or bindings[candidate['path']]!=candidate['iteration']:
            if Path(candidate['path']).name.startswith('candidate_'):
                raise ValueError('候选文件缺少对应保存记录，请显式指定候选')
            row.update(eligible=False, reason='no_matching_save_record')
        else:
            samples=[v for i,v in by_iteration.items() if candidate['iteration']-window+1<=i<=candidate['iteration']]
            completed=sum(v['completed'] for v in samples)
            successes=sum(v['successes'] for v in samples)
            row.update(eligible=completed>0, reason=None if completed else 'no_complete_episodes',
                       completed=completed,successes=successes,
                       success_rate=successes/completed if completed else None,
                       iterations=[v['iteration'] for v in samples],
                       window_start_step=samples[0]['window_start_step'],window_end_step=samples[-1]['window_end_step'])
        mapped.append(row)
    available=[r for r in mapped if r['eligible']]
    if not available:
        retained = [c for c in checkpoints if Path(c['path']).name.startswith('candidate_')]
        if retained:
            return retained, dict(method='retained_candidates_without_new_window_ranking', mapping=mapped,
                warning='本次筛选窗口无完成回合；仅评估已有训练候选，不将零分母当成功率排名。')
        raise ValueError('没有已完成回合且保存记录可关联的候选；不能按零样本排名，请显式指定候选')
    selected=[]
    groups=min(candidates,len(available))
    for group in range(groups):
        segment=available[group*len(available)//groups:(group+1)*len(available)//groups]
        best=max(segment,key=lambda r:(r['success_rate'],r['completed'],-r['iteration']))
        selected.append(next(c for c in checkpoints if c['path']==best['path']))
    final=checkpoints[-1]
    if final not in selected:
        selected.append(final)
    return selected,dict(method='temporal_groups_complete_episode_counts',window_iterations=window,
        mapping=mapped,requested_groups=candidates,final_checkpoint_included_for_coverage=True,
        source_files=[dict(path=str(p),sha256=digest(p)) for p in (episode_file,checkpoint_file)],
        ranking_rule='每时段按窗口sum(successes)/sum(completed)提名；同分取样本更多者，再取较早iter；零分母不排名，末checkpoint额外纳入覆盖。',
        warning='仅rank 0训练回合，回合可能跨策略更新；窗口截至保存iteration，不使用未来样本。checkpoint为更新后权重，不代表冻结评估成绩。')


def nominate(run, candidates, window, manual, strategy="saved"):
    """文件编号只用于映射日志横轴和时段，不作为模型质量分数。"""
    if strategy not in ("saved", "temporal"):
        raise ValueError("unknown nomination strategy")
    import torch
    checkpoints = []
    candidate_dir = run/'training-candidates'
    candidate_paths = list(candidate_dir.glob('candidate_*.pt'))
    manifest_path = candidate_dir/'candidates.json'
    if candidate_paths or manifest_path.exists():
        if not manifest_path.is_file():
            raise ValueError('训练候选缺少candidates.json；请先完成训练并恢复完整候选记录')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        if manifest.get('schema_version') != 1 or manifest.get('status') != 'complete':
            raise ValueError('训练候选尚未正常结束；不读取运行中或中断状态的候选')
        declared = [allowed_checkpoint(row['checkpoint'], run) for row in manifest['candidates']]
        actual = {path.resolve() for path in candidate_paths}
        if (any(path.parent != candidate_dir for path in declared)
                or len(set(declared)) != len(declared) or set(declared) != actual):
            raise ValueError('训练候选manifest与现存文件不一致；请先核查训练输出')
    saved_only = not manual and strategy == 'saved'
    if saved_only:
        if not manifest_path.is_file() or not candidate_paths:
            raise ValueError('没有完整的动态候选；请使用 --checkpoints 显式指定，或 --nomination temporal 启用旧时段提名')
        manual = declared
    paths = list(manual) if manual else list(run.glob('model_*.pt')) + candidate_paths
    for path in paths:
        if not re.fullmatch(r'(model|candidate)_\d+\.pt', path.name):
            continue
        path = allowed_checkpoint(path, run)
        if not path.is_file():
            continue
        # 只加载当前用户训练产物；不允许任意pickle执行作为兼容回退。
        data = torch.load(path, map_location='cpu', weights_only=True)
        iteration = data['iter']
        if isinstance(iteration, bool) or not isinstance(iteration, int):
            raise ValueError(f'{path}: iter必须为整数')
        if iteration != int(path.stem.rsplit('_', 1)[1]):
            raise ValueError(f'{path}: 文件轮次与checkpoint iter不一致')
        checkpoints.append(dict(path=str(path.resolve()), iteration=iteration, sha256=digest(path)))
    checkpoints.sort(key=lambda c:(c['iteration'], c['path']))
    if not checkpoints:
        raise ValueError('训练目录没有model_<轮次>.pt或training-candidates/candidate_<轮次>.pt')
    if manual:
        requested = {str(p.resolve()) for p in manual}
        selected = [c for c in checkpoints if c['path'] in requested]
        missing = requested - {c['path'] for c in selected}
        if missing:
            raise ValueError(f'手动候选不属于该训练目录的有效checkpoint: {sorted(missing)}')
        return selected, dict(method='saved_training_candidates' if saved_only else 'manual', checkpoints=checkpoints)
    complete_episode_nomination = nominate_complete_episodes(run, checkpoints, candidates, window)
    if complete_episode_nomination is not None:
        return include_saved_candidates(complete_episode_nomination, checkpoints)
    events = sorted(run.glob('events.out.tfevents.*'))
    if not events:
        raise ValueError('缺少同目录TensorBoard日志；请用--checkpoints显式指定候选，不自动伪造训练排名')
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    series = {'Train/mean_reward':{}, 'Train/mean_episode_length':{}}
    for event in events:
        acc = EventAccumulator(str(event), size_guidance={'scalars':0}); acc.Reload()
        for tag in series:
            if tag not in acc.Tags().get('scalars', []):
                continue
            for item in acc.Scalars(tag):
                if not math.isfinite(item.value):
                    continue
                if item.step in series[tag]:
                    raise ValueError(f'{tag}在step={item.step}重复；可能有恢复训练日志，请显式指定候选')
                series[tag][item.step] = item.value
    if not all(series.values()):
        raise ValueError('缺少Train/mean_reward或Train/mean_episode_length；请用--checkpoints显式指定候选')
    mapped = []
    for c in checkpoints:
        row = dict(c)
        for tag, values in series.items():
            points = [(i,v) for i,v in values.items() if c['iteration']-window+1 <= i <= c['iteration']]
            row[tag] = dict(mean=statistics.mean(v for _,v in points) if points else None,
                            count=len(points), steps=sorted(i for i,_ in points))
        mapped.append(row)
    available = [c for c in mapped if all(c[t]['count'] for t in series)]
    if not available:
        raise ValueError('checkpoint附近没有可匹配日志；请显式指定候选')
    # 按保存时间分组，每组保留平滑奖励最佳者，防止只提名相邻的同一小段。
    selected = []
    groups = min(candidates, len(available))
    for group in range(groups):
        segment = available[group*len(available)//groups:(group+1)*len(available)//groups]
        best = max(segment, key=lambda c:(c['Train/mean_reward']['mean'],
                                         c['Train/mean_episode_length']['mean'], -c['iteration']))
        selected.append(next(c for c in checkpoints if c['path']==best['path']))
    final = checkpoints[-1]
    if final not in selected:
        selected.append(final)
    return include_saved_candidates((selected, dict(method='temporal_groups_smoothed_training_reward', window_iterations=window,
        requested_groups=candidates, mapping=mapped, event_files=[dict(path=str(p),sha256=digest(p)) for p in events],
        warning='日志均值是更新前采集且可跨策略版本的最近100个结束回合；model_it为更新后权重。这里只提名，不是冻结checkpoint成绩；旧Metrics/success_rate不参与。')), checkpoints)


def start_evaluation(candidate, args, output, seeds, stage):
    candidate = {key: candidate[key] for key in ('path','iteration','sha256')}
    if digest(Path(candidate['path'])) != candidate['sha256']:
        raise ValueError('checkpoint在选择过程中被修改，请等待训练结束后重跑')
    target = output/stage/(Path(candidate['path']).stem+'_'+candidate['sha256'][:12])
    target.mkdir(parents=True)
    cmd = [sys.executable, str(Path(__file__).with_name('evaluate_policy.py')),
           '--checkpoint',candidate['path'],'--task',args.task,'--num_envs',str(args.num_envs),
           '--seeds',*map(str,seeds),'--device',args.device,'--viz','none','--info',
           '--output-dir',str(target)]
    if args.horizon_s is not None:
        cmd += ['--horizon-s',str(args.horizon_s)]
    for flag, values in [('--initial-cart-velocity-range',args.initial_cart_velocity_range),
                         ('--initial-pole-velocity-range',args.initial_pole_velocity_range)]:
        if values is not None:
            cmd += [flag,*map(str,values)]
    print(f"[{stage}] {Path(candidate['path']).name}", flush=True)
    log = (target/'console.log').open('w', encoding='utf-8')
    try:
        process = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == 'nt' else 0,
            start_new_session=os.name != 'nt')
    except BaseException:
        log.close()
        raise
    return dict(candidate=candidate, target=target, process=process, log=log,
                started=time.perf_counter())


def finish_evaluation(job, args, seeds):
    candidate, target, result = job['candidate'], job['target'], job['process']
    job['log'].close()
    reports = list(target.glob('evaluation-*/report.json'))
    if result.returncode != 0 or len(reports)!=1:
        raise RuntimeError(f'评估未完成: {target}/console.log (exit={result.returncode})')
    path = reports[0]; report = json.loads(path.read_text(encoding='utf-8'))
    if (report['status']!='complete' or report['summary']['status']!='complete'
        or report['checkpoint_sha256']!=candidate['sha256'] or report['seeds']!=seeds
        or report['num_envs']!=args.num_envs or report.get('task')!=args.task):
        raise ValueError(f'报告不完整或身份不匹配: {path}')
    episodes = [e for c in report['cohorts'] for e in c['episodes']]
    if len(episodes)!=len(seeds)*args.num_envs:
        raise ValueError('回合数量不匹配')
    successes = [e for e in episodes if e['success']]
    speed = statistics.mean(e['cart_mean_abs_velocity_m_s'] for e in successes) if successes else None
    drift = statistics.mean(abs(e['cart_net_displacement_m']) for e in successes) if successes else None
    return dict(**candidate, task=report['task'], report=str(path), summary=report['summary'],
                successful_episode_mean_abs_velocity_m_s=speed,
                successful_episode_mean_abs_net_displacement_m=drift,
                env_config_sha256=digest(path.parent/'env.yaml'), code=report['code'],
                evaluation_wall_s=time.perf_counter()-job['started'])


def stop_evaluations(jobs):
    """只终止本次启动的评估进程组，不触碰其他训练或选择任务。"""
    errors = []
    for job in jobs:
        process = job['process']
        try:
            if process.poll() is None:
                if os.name == 'nt':
                    subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   check=False, timeout=5)
                else:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
        except Exception as exc:
            errors.append(f'PID {process.pid}: {exc}')
    for job in jobs:
        process = job['process']
        try:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if os.name != 'nt':
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                else:
                    process.kill()
                process.wait(timeout=5)
        except Exception as exc:
            errors.append(f'PID {process.pid}: {exc}')
        finally:
            job['log'].close()
    if errors:
        raise RuntimeError('评估进程清理异常，请核查这些PID: ' + '; '.join(errors))


def evaluate_candidates(candidates, args, output, seeds, stage, on_result, workers):
    """有界并行；失败或 Ctrl+C 清理所有仍在运行的本次子进程。"""
    pending = iter(enumerate(candidates))
    active = []
    results = {}
    try:
        exhausted = False
        while active or not exhausted:
            while not exhausted and len(active) < workers:
                item = next(pending, None)
                if item is None:
                    exhausted = True
                    break
                index, candidate = item
                job = start_evaluation(candidate, args, output, seeds, stage)
                job['index'] = index
                active.append(job)
            completed = [job for job in active if job['process'].poll() is not None]
            for job in completed:
                row = finish_evaluation(job, args, seeds)
                on_result(row)
                results[job['index']] = row
                active.remove(job)
                print(f"[{stage} complete] {Path(row['path']).name} "
                      f"wall={row['evaluation_wall_s']:.1f}s", flush=True)
            if active and not completed:
                time.sleep(0.05)
        return [results[index] for index in range(len(candidates))]
    finally:
        original_error = sys.exc_info()[1]
        try:
            stop_evaluations(active)
        except Exception as cleanup_error:
            if original_error is not None:
                raise RuntimeError(f'{type(original_error).__name__}: {original_error}; '
                                   f'cleanup: {cleanup_error}') from original_error
            raise


def evaluate(candidate, args, output, seeds, stage):
    return evaluate_candidates([candidate], args, output, seeds, stage, lambda row: None, 1)[0]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--task',default='Hexchip-Cartpole-Real-V2-v0',
                   choices=['Hexchip-Cartpole-Real-V2-v0', 'Hexchip-Cartpole-Real-V2-Window16-v0',
                            'Hexchip-Cartpole-Real-V2-Window16-Tau0-v0',
                            'Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0'],
                   help='候选及holdout统一评估任务；必须显式匹配目标观测和执行器配置，默认保持旧任务')
    p.add_argument('--checkpoints',type=Path,nargs='+',help='显式候选；绕过训练日志提名')
    p.add_argument('--nomination', choices=['saved','temporal'], default='saved',
                   help='默认仅评估已保存动态候选；temporal 显式启用时段提名、末模型及动态候选')
    p.add_argument('--candidates',type=int,default=3,help='仅 temporal 模式的训练时段数，不是动态候选数量')
    p.add_argument('--parallel-workers',type=int,default=1,
                   help='同时评估的候选数；默认1保持串行，共享device，显存不足或争用时减小')
    p.add_argument('--window',type=int,default=20,help='提名用日志窗口，单位为训练迭代')
    p.add_argument('--num_envs',type=int,default=32,help='每候选每种子的完整回合数')
    p.add_argument('--seeds',type=int,nargs='+',default=[101,202])
    p.add_argument('--holdout-seeds',type=int,nargs='+',default=[1001,2002])
    p.add_argument('--device',default='cuda:0')
    p.add_argument('--horizon-s',type=float)
    p.add_argument('--initial-cart-velocity-range',type=float,nargs=2)
    p.add_argument('--initial-pole-velocity-range',type=float,nargs=2)
    p.add_argument('--output-dir',type=Path,default=Path('logs/checkpoint-selection'))
    args = p.parse_args()
    if args.horizon_s is not None and (not math.isfinite(args.horizon_s) or args.horizon_s<=0):
        p.error('horizon-s必须为有限正数')
    for values in (args.initial_cart_velocity_range,args.initial_pole_velocity_range):
        if values is not None and (not all(math.isfinite(v) for v in values) or values[0]>values[1]):
            p.error('初速度区间必须为有序有限数值')
    if min(args.candidates,args.window,args.num_envs,args.parallel_workers)<=0:
        p.error('数量和窗口必须为正')
    all_seeds = args.seeds+args.holdout_seeds
    if any(s<0 for s in all_seeds) or len(set(all_seeds))!=len(all_seeds):
        p.error('所有种子必须非负、互不重复；选择与复核种子必须分离')
    output = args.output_dir.resolve()/('selection-'+uuid.uuid4().hex)
    output.mkdir(parents=True)
    report = dict(status='running',scope='已评估候选中的最佳；不是全局最优或真机验收',
                  options={k:str(v) if isinstance(v,Path) else [str(p) for p in v] if k=='checkpoints' and v else v
                           for k,v in vars(args).items()},evaluated=[])
    dest = output/'selection.json'; save(dest,report)
    print(f'[OUTPUT] {dest}',flush=True)
    try:
        candidates, nomination = nominate(args.run_dir.resolve(strict=True),args.candidates,args.window,args.checkpoints,strategy=args.nomination)
        report['nomination']=nomination; report['candidates']=candidates; save(dest,report)
        def record_result(row):
            if report['evaluated']:
                first = report['evaluated'][0]
                if row['env_config_sha256']!=first['env_config_sha256'] or row['code']!=first['code']:
                    raise ValueError('候选之间配置或代码发生变化，不允许混合排名')
            report['evaluated'].append(row); save(dest,report)
        report['evaluated'] = evaluate_candidates(
            candidates, args, output, args.seeds, 'selection', record_result, args.parallel_workers)
        save(dest, report)
        ranking = sorted(report['evaluated'],key=lambda r:(-r['summary']['success_rate'],
            r['successful_episode_mean_abs_velocity_m_s'] if r['successful_episode_mean_abs_velocity_m_s'] is not None else math.inf,
            r['successful_episode_mean_abs_net_displacement_m'] if r['successful_episode_mean_abs_net_displacement_m'] is not None else math.inf,
            r['iteration']))
        report['ranking_rule']='成功率降序；相同则成功回合平均绝对车速升序、绝对首尾位移升序；仍相同取较早iter。失败不因存活时间短而在速度排名获益。'
        report['ranking']=ranking
        if ranking[0]['summary']['successes']==0:
            raise ValueError('所有候选均无成功回合，不能推荐可用模型')
        report['selected']=ranking[0]; save(dest,report)
        report['holdout']=evaluate(ranking[0],args,output,args.holdout_seeds,'holdout')
        if report['holdout']['code'] != report['selected']['code']:
            raise ValueError('复核时评估代码发生变化，请使用固定版本重跑')
        report['holdout_note']='独立种子只复核，不用来再次选模型；反复使用本集合调参后不能再称为未见数据。相同seed不保证不同策略的后续自动reset随机流完全相同。'
        report['status']='complete'; save(dest,report)
        print('SELECTED_CHECKPOINT '+report['selected']['path'],flush=True)
        print('结果与复核: '+str(dest),flush=True)
    except BaseException as exc:
        report['status']='error'; report['error']=f'{type(exc).__name__}: {exc}'; save(dest,report)
        raise


if __name__=='__main__':
    main()

