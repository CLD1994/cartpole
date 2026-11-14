"""冻结策略的完整回合评估；不训练、不导出策略、不接触真机。"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import uuid

from episode_metrics import EpisodeMetrics, run_cohort, summarize


def parse_args(argv=None, *, add_launcher_args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--task', default='Hexchip-Cartpole-Real-V2-v0',
                        choices=['Hexchip-Cartpole-Real-V2-v0'])
    parser.add_argument('--num_envs', type=int, default=100, help='每个种子的完整回合数量')
    parser.add_argument('--seeds', type=int, nargs='+', default=[101, 202, 303, 404, 505])
    parser.add_argument('--horizon-s', type=float, help='仅评估实例的回合时长；默认保留训练配置')
    parser.add_argument('--render-interval', type=int, default=None)
    parser.add_argument('--output-dir', type=Path, default=Path('logs/evaluation'))
    add_launcher_args(parser)
    args = parser.parse_args(argv)
    args.physics = 'isaacsim_physx'
    if args.num_envs <= 0 or any(seed < 0 for seed in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('num_envs 必须为正，种子必须非负且不能重复')
    if args.horizon_s is not None and (not math.isfinite(args.horizon_s) or args.horizon_s <= 0):
        parser.error('horizon-s 必须为有限正数')
    if args.render_interval is not None and args.render_interval <= 0:
        parser.error('render-interval 必须为正')
    return args


def make_recorder_type(base, *, snapshot, consume, before_reset=lambda ids: None):
    """在公开 post_step 钩子采样，早于自动 reset；不向数据集写入张量。"""
    class EvaluationRecorder(base):
        def record_pre_reset(self, env_ids):
            before_reset(env_ids)
            return None, None

        def record_post_step(self):
            consume(snapshot(self._env))
            return None, None
    return EvaluationRecorder


def snapshot_environment(env):
    """读取真实状态而非有噪声观测；只接受当前 v2 两关节语义。"""
    import torch
    robot = env.scene['robot']
    cart_ids, _ = robot.find_joints('slider_to_cart')
    pole_ids, _ = robot.find_joints('cart_to_pole')
    if len(cart_ids) != 1 or len(pole_ids) != 1:
        raise ValueError('评估器要求唯一的小车和摆杆关节')
    cart, pole = cart_ids[0], pole_ids[0]
    positions, velocities = robot.data.joint_pos.torch, robot.data.joint_vel.torch
    theta = torch.remainder(positions[:, pole] + math.pi, 2*math.pi) - math.pi
    state = torch.stack((positions[:,cart],theta,velocities[:,cart],velocities[:,pole]),dim=1)
    actions = env.action_manager.action
    if actions.shape[1] != 1:
        raise ValueError('评估器要求一个速度指令动作')
    manager = env.termination_manager
    return dict(states=state.detach().cpu().tolist(), actions=actions[:,0].detach().cpu().tolist(),
                terminated=manager.terminated.detach().cpu().tolist(),
                truncated=manager.time_outs.detach().cpu().tolist(),
                reasons={name:manager.get_term(name).detach().cpu().tolist() for name in manager.active_terms})


def write_report(path, report):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)



def refresh_summary(report, status):
    """全局状态覆盖局部cohort状态；部分种子完成不能发布完整成功率。"""
    summary = summarize(report['cohorts'])
    summary['status'] = status
    if status != 'complete':
        summary.update(success_rate=None,wilson_95_low=None,wilson_95_high=None)
    report.update(status=status,summary=summary)


def persist_progress(path, report):
    refresh_summary(report,'running')
    write_report(path,report)


def persist_error(path, report, active, error):
    if active[0] is not None:
        report['interrupted_cohort'] = active[0].report()
    refresh_summary(report,'error')
    report['error'] = dict(type=type(error).__name__,message=str(error))
    write_report(path,report)


@contextmanager
def reported_run(path, report, *, active, emit=print):
    """必须嵌套在launch_simulation内部，确保Kit可能退出进程前已落盘。

    退出码只表示采集完整；不可用cohort局部完成替代全部请求种子完成。
    """
    try:
        yield
    except BaseException as error:
        persist_error(path,report,active,error)
        if isinstance(error,SystemExit) and error.code in (None,0):
            raise SystemExit(1) from error
        raise
    else:
        cohorts = report['cohorts']
        complete = ([c['seed'] for c in cohorts] == report['seeds']
                    and all(c['status']=='complete'
                            and c['requested']==report['num_envs']
                            and c['completed']==report['num_envs']
                            and len(c['episodes'])==report['num_envs'] for c in cohorts))
        refresh_summary(report,'complete' if complete else 'incomplete')
        write_report(path,report)
        emit('POLICY_EVALUATION '+json.dumps(report['summary'],ensure_ascii=False))
        if not complete:
            raise SystemExit(1)


def main(argv=None):
    import warp as wp
    wp.config.enable_backward = False
    import cartpole.tasks  # noqa: F401
    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab.managers import RecorderTerm, RecorderTermCfg, RecorderManagerBaseCfg, DatasetExportMode
    from isaaclab.utils.io import dump_yaml
    from isaaclab_tasks.utils import resolve_task_config
    from isaaclab_rl.rsl_rl import (RslRlVecEnvWrapper, check_rsl_rl_version,
                                   create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg)

    args = parse_args(argv,add_launcher_args=add_launcher_args)
    checkpoint = args.checkpoint.resolve(strict=True)
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    output = args.output_dir.resolve() / ('evaluation-'+uuid.uuid4().hex)
    output.mkdir(parents=True,exist_ok=False)
    report = dict(schema_version=1,status='starting',checkpoint=str(checkpoint),checkpoint_sha256=digest,
                  task=args.task,seeds=args.seeds,num_envs=args.num_envs,cohorts=[],
                  scope='当前配置下的独立仿真评估；不是旧训练环境重建、性能基准或真机验收',
                  sample_scope='每槽位从显式reset起的第一个回合；状态在自动reset前、每控制步末采样',
                  recorder_note='Isaac Lab 3.0 recorder会额外计算观测；相同seed不保证与普通play随机数序列相同，不能用于RTF比较',
                  state_order=['x_m','theta_rad','xdot_m_s','thetadot_rad_s'])
    project = Path(__file__).resolve().parents[2]
    def git(*arguments):
        completed = subprocess.run(['git','-C',str(project),*arguments],capture_output=True,text=True)
        return completed.stdout.strip() if completed.returncode == 0 else None
    report['code'] = dict(commit=git('rev-parse','HEAD'),dirty_status=git('status','--short'))
    write_report(output/'report.json',report)
    print(f'[OUTPUT] {output}',flush=True)
    active = [None]
    try:
        env_cfg,agent_cfg = resolve_task_config(args.task,'rsl_rl_cfg_entry_point',play_mode=False,overrides=[])
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg,check_rsl_rl_version())
        env_cfg.scene.num_envs = args.num_envs
        env_cfg.sim.device = args.device
        env_cfg.seed = agent_cfg.seed = args.seeds[0]
        env_cfg.log_dir = None
        if args.horizon_s is not None:
            env_cfg.episode_length_s = args.horizon_s
        if args.render_interval is not None:
            env_cfg.sim.render_interval = args.render_interval
        with launch_simulation(env_cfg,args):
            with reported_run(output/'report.json',report,active=active,emit=lambda text:print(text,flush=True)):
                # 保存解析后的物理配置，不把诊断recorder写成训练模型的一部分。
                dump_yaml(str(output/'env.yaml'),env_cfg)
                def consume(data):
                    if active[0] is not None:
                        active[0].record(**data)
                def before_reset(env_ids):
                    if active[0] is not None:
                        if hasattr(env_ids, 'detach'):
                            env_ids = env_ids.detach().cpu().tolist()
                        active[0].check_reset(env_ids)
                recorder_type = make_recorder_type(RecorderTerm,snapshot=snapshot_environment,
                                                   consume=consume,before_reset=before_reset)
                recorder_cfg = RecorderManagerBaseCfg()
                recorder_cfg.dataset_export_mode = DatasetExportMode.EXPORT_NONE
                recorder_cfg.export_in_record_pre_reset = False
                recorder_cfg.export_in_close = False
                recorder_cfg.evaluation = RecorderTermCfg(class_type=recorder_type)
                env_cfg.recorders = recorder_cfg
                raw = gym.make(args.task,cfg=env_cfg)
                try:
                    env = RslRlVecEnvWrapper(raw,clip_actions=agent_cfg.clip_actions)
                    agent_cfg.device = str(env.unwrapped.device)
                    dump_yaml(str(output/'agent.yaml'),agent_cfg)
                    runner = create_rsl_rl_runner(env,agent_cfg,log_dir=None)
                    runner.load(str(checkpoint))
                    policy = runner.get_inference_policy(device=env.unwrapped.device)
                    dt = env.unwrapped.step_dt
                    report.update(step_dt_s=dt,physics_dt_s=env.unwrapped.physics_dt,
                                  horizon_s=env_cfg.episode_length_s,device=str(env.unwrapped.device),
                                  physics_backend=type(env_cfg.sim.physics).__name__,
                                  visualization=getattr(args,'visualizer',None),
                                  render_interval=env_cfg.sim.render_interval)
                    for seed in args.seeds:
                        active[0] = None
                        raw.reset(seed=seed)
                        policy.reset(torch.ones(args.num_envs,dtype=torch.bool,device=env.unwrapped.device))
                        obs = env.get_observations()
                        metrics = EpisodeMetrics(num_envs=args.num_envs,seed=seed,dt=dt)
                        active[0] = metrics
                        def step():
                            nonlocal obs
                            obs,_,dones,_ = env.step(policy(obs))
                            policy.reset(dones)
                        with torch.inference_mode():
                            cohort = run_cohort(metrics,step=step,max_steps=math.ceil(env_cfg.episode_length_s/dt)+2)
                        active[0] = None
                        report['cohorts'].append(cohort)
                        persist_progress(output/'report.json',report)
                        print(f"[SEED {seed}] {cohort['completed']}/{args.num_envs} complete, successes={cohort['successes']}",flush=True)
                        if cohort['status'] != 'complete':
                            break
                finally:
                    raw.close()
        return 0
    except BaseException as error:
        # Kit may raise SystemExit during shutdown. Never overwrite a terminal report
        # with that expected exit; ordinary startup/shutdown failures still count as errors.
        expected_exit = (isinstance(error,SystemExit)
                         and ((report['status']=='complete' and error.code in (None,0))
                              or (report['status']=='incomplete' and error.code==1)))
        if not expected_exit and report['status'] != 'error':
            persist_error(output/'report.json',report,active,error)
        raise


if __name__ == '__main__':
    raise SystemExit(main())
