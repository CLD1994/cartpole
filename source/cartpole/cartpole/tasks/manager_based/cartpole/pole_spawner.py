"""V2 摆杆的场景创建入口。"""
from isaaclab.sim.spawners.from_files import spawn_from_usd
from isaaclab.sim.utils import clone

from cartpole.device_profile import load_profile, model_parameters
from cartpole.pole_geometry import apply_pole_geometry


@clone
def spawn_v2_cartpole(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """先完成源环境的杆几何，再由 Isaac Lab 复制到其余环境。"""
    root = spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    apply_pole_geometry(root.GetStage(), prim_path, model_parameters(load_profile()))
    return root
