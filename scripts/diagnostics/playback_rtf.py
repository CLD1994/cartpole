"""Isaac Lab 3.0 单环境回放计时。不导出、保存或修改 checkpoint。"""
import argparse
import hashlib
import json
from pathlib import Path

from playback_timing import HostPhaseTimer, measure_playback


def load_policy(env, agent_cfg, checkpoint, *, create_runner):
    """在实际仿真设备构造并加载策略；不启用训练日志或模型导出。"""
    agent_cfg.device = str(env.unwrapped.device)
    runner = create_runner(env, agent_cfg, log_dir=None)
    runner.load(str(checkpoint))
    return runner.get_inference_policy(device=env.unwrapped.device)


def parse_args(argv=None, *, add_launcher_args):
    """解析诊断选项；启动器参数由调用方提供。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default='Hexchip-Cartpole-Real-V2-v0')
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--steps', type=int, default=5000)
    parser.add_argument('--warmup', type=int, default=500)
    parser.add_argument('--real-time', action='store_true')
    parser.add_argument('--breakdown', action='store_true', help='记录主机调用分段耗时，不等于GPU核耗时')
    add_launcher_args(parser)
    parser.add_argument('--render-interval', type=int, default=None,
                        help='每隔多少个物理步更新显示；默认保留任务配置，不改变物理或控制周期')
    args = parser.parse_args(argv)
    # 显式固定同一物理后端，不能让有无界面改变物理实现。
    args.physics = 'isaacsim_physx'
    if args.seed < 0 or args.steps <= 0 or args.warmup < 0:
        parser.error('seed/warmup 不得为负，steps 必须为正')
    if args.render_interval is not None and args.render_interval <= 0:
        parser.error('render-interval 必须为正整数')
    return args


def main(argv=None):
    # 与项目 train/play 入口一致，在任务导入前初始化 Warp 配置。
    import warp as wp
    wp.config.enable_backward = False
    import cartpole.tasks  # noqa: F401
    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab_tasks.utils import resolve_task_config
    from isaaclab_rl.rsl_rl import (
        RslRlVecEnvWrapper, check_rsl_rl_version, create_rsl_rl_runner,
        handle_deprecated_rsl_rl_cfg,
    )

    args = parse_args(argv, add_launcher_args=add_launcher_args)
    checkpoint = args.checkpoint.resolve(strict=True)
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    version = check_rsl_rl_version()
    # 与官方 play 默认一致：使用 play_mode，不恢复 checkpoint 目录中的 env.yaml。
    env_cfg, agent_cfg = resolve_task_config(args.task, 'rsl_rl_cfg_entry_point',
                                             play_mode=True, overrides=[])
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, version)
    agent_cfg.seed = args.seed
    env_cfg.seed = args.seed
    env_cfg.scene.num_envs = 1
    env_cfg.sim.device = args.device
    if args.render_interval is not None:
        env_cfg.sim.render_interval = args.render_interval
    # 禁用录像/轨迹记录，不把 checkpoint 所在目录作为输出目录。
    env_cfg.log_dir = None
    with launch_simulation(env_cfg, args):
        raw_env = gym.make(args.task, cfg=env_cfg)
        try:
            env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
            policy = load_policy(env, agent_cfg, checkpoint, create_runner=create_rsl_rl_runner)
            obs = env.get_observations()
            phases = HostPhaseTimer()
            def step():
                nonlocal obs
                if args.breakdown:
                    action = phases.call('policy', policy, obs)
                    obs, _, dones, _ = phases.call('environment', env.step, action)
                    phases.call('policy_recurrent_reset', policy.reset, dones)
                else:
                    obs, _, dones, _ = env.step(policy(obs))
                    policy.reset(dones)
            device = torch.device(env.unwrapped.device)
            def synchronize():
                if device.type == 'cuda':
                    torch.cuda.synchronize(device)
            with torch.inference_mode():
                result = measure_playback(step, dt=env.unwrapped.step_dt,
                    steps=args.steps, warmup=args.warmup, real_time=args.real_time,
                    synchronize=synchronize, on_measurement_start=phases.clear)
            result.update(checkpoint=str(checkpoint), checkpoint_sha256=digest,
                task=args.task, seed=args.seed, num_envs=1,
                physics_dt_s=env.unwrapped.physics_dt,
                render_interval=env_cfg.sim.render_interval,
                visualization=getattr(args, 'visualizer', None), device=str(device),
                physics_backend=type(env_cfg.sim.physics).__name__)
            if args.breakdown:
                result['host_call_timings'] = phases.summary()
                result['timing_scope'] = 'host calls; environment includes physics, episode reset and rendering; not GPU kernel attribution'
            print('PLAYBACK_TIMING ' + json.dumps(result, ensure_ascii=False))
        finally:
            raw_env.close()


if __name__ == '__main__':
    main()
