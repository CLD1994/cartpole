"""V2 摆杆场景创建入口；配置导入阶段不加载 USD 或仿真执行模块。"""


def spawn_v2_cartpole(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """运行时启动后才加载 spawner；源环境完成后由官方 clone 复制。"""
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from isaaclab.sim.utils import clone
    from cartpole.device_profile import load_profile, model_parameters
    from cartpole.pole_geometry import apply_pole_geometry

    @clone
    def spawn_source(path, config, translation=None, orientation=None, **options):
        root = spawn_from_usd(path, config, translation, orientation, **options)
        apply_pole_geometry(root.GetStage(), path, model_parameters(load_profile()))
        return root

    return spawn_source(prim_path, cfg, translation, orientation, **kwargs)
