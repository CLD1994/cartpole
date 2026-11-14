"""Isaac Lab 3.0 零动作环境基线；不加载策略，不修改任务默认配置。"""
import argparse
import json
import math

from playback_timing import measure_playback


def parse_args(argv=None, *, add_launcher_args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default='Isaac-Cartpole')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--steps', type=int, default=5000)
    parser.add_argument('--warmup', type=int, default=500)
    parser.add_argument('--physics-dt', type=float, default=.001)
    parser.add_argument('--decimation', type=int, default=2)
    parser.add_argument('--render-interval', type=int, default=20)
    add_launcher_args(parser)
    args = parser.parse_args(argv)
    if (args.seed < 0 or args.steps <= 0 or args.warmup < 0 or args.decimation <= 0
            or args.render_interval <= 0 or not math.isfinite(args.physics_dt) or args.physics_dt <= 0):
        parser.error('seed/warmup 不得为负；steps、physics-dt、decimation、render-interval 必须为正')
    args.physics = 'isaacsim_physx'
    return args


def configure_baseline(cfg, args):
    """只修改本次创建的配置对象，不保存配置或改写仓库任务。"""
    cfg.seed = args.seed
    cfg.scene.num_envs = 1
    cfg.sim.device = args.device
    cfg.sim.dt = args.physics_dt
    cfg.decimation = args.decimation
    cfg.sim.render_interval = args.render_interval
    cfg.log_dir = None


def measure_zero_actions(env, *, make_zeros, measure, synchronize, steps, warmup):
    """复用设备上固定零动作，保留环境的奖励、终止、自动重置和观察计算。"""
    base = env.unwrapped
    action = make_zeros((base.num_envs, base.action_manager.total_action_dim), device=base.device)
    env.reset()
    return measure(lambda: env.step(action), dt=base.step_dt, steps=steps,
                   warmup=warmup, synchronize=synchronize)


def main(argv=None):
    import warp as wp
    wp.config.enable_backward = False
    import cartpole.tasks  # noqa: F401
    import isaaclab_tasks.core.cartpole  # noqa: F401
    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab_tasks.utils import resolve_task_config

    args = parse_args(argv, add_launcher_args=add_launcher_args)
    cfg, _ = resolve_task_config(args.task, 'rsl_rl_cfg_entry_point', play_mode=True, overrides=[])
    configure_baseline(cfg, args)
    with launch_simulation(cfg, args):
        env = gym.make(args.task, cfg=cfg)
        try:
            device = torch.device(env.unwrapped.device)
            def synchronize():
                if device.type == 'cuda':
                    torch.cuda.synchronize(device)
            with torch.inference_mode():
                result = measure_zero_actions(env, make_zeros=torch.zeros, measure=measure_playback,
                    synchronize=synchronize, steps=args.steps, warmup=args.warmup)
            result.update(task=args.task, seed=args.seed, num_envs=env.unwrapped.num_envs,
                action='constant_zero', physics_dt_s=env.unwrapped.physics_dt,
                decimation=cfg.decimation, render_interval=cfg.sim.render_interval,
                visualization=getattr(args, 'visualizer', None), device=str(device),
                physics_backend=type(cfg.sim.physics).__name__,
                scope='zero-action environment baseline; includes automatic resets; no policy inference; not equivalent dynamics across tasks')
            print('ENV_STEP_TIMING ' + json.dumps(result, ensure_ascii=False))
        finally:
            env.close()


if __name__ == '__main__':
    main()
