"""Isaac Lab 3.0 / Isaac Sim PhysX 的有限时长摆杆检查；不加载策略或串口。"""
import argparse
import json
import math
import statistics


def measure_period(times, angles, expected, tolerance=0.03):
    """由相邻正向过零时刻测量完整周期；输入角度相对下垂方向。"""
    if len(times) != len(angles) or len(times) < 2:
        raise ValueError("时间和角度须等长且至少有两个样本")
    if not all(math.isfinite(x) for x in [*times, *angles, expected]) or expected <= 0:
        raise ValueError("样本和理论周期必须为有限值，周期须为正")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("仿真时间必须严格递增")
    crossings = []
    for t0, t1, a0, a1 in zip(times, times[1:], angles, angles[1:]):
        if a0 < 0 <= a1:
            crossings.append(t0 + (t1 - t0) * (-a0) / (a1 - a0))
    if len(crossings) < 4:
        raise ValueError("不足四次正向过零，不能确认三个完整周期")
    periods = [b - a for a, b in zip(crossings, crossings[1:])]
    error = max(abs(p / expected - 1) for p in periods)
    return {"periods_s": periods, "median_period_s": statistics.median(periods),
            "expected_period_s": expected, "max_relative_error": error,
            "passed": error <= tolerance}


def spawn_fixed_cart(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """仅诊断场景：小车滑动关节改为固定连接，关闭碰撞和杆体阻尼。"""
    from pxr import PhysxSchema, Usd, UsdPhysics
    from isaaclab.sim.utils import make_uninstanceable
    from cartpole.tasks.manager_based.cartpole.pole_spawner import spawn_v2_cartpole

    root = spawn_v2_cartpole(prim_path, cfg, translation, orientation, **kwargs)
    stage = root.GetStage()
    make_uninstanceable(prim_path, stage)
    joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/slider/slider_to_cart")
    if joint.GetPrim().HasAPI(UsdPhysics.DriveAPI, "linear"):
        joint.GetPrim().RemoveAPI(UsdPhysics.DriveAPI, "linear")
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            physx_root = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            physx_root.CreateSleepThresholdAttr(0.0)
            physx_root.CreateStabilizationThresholdAttr(0.0)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
    pole = stage.GetPrimAtPath(f"{prim_path}/pole")
    PhysxSchema.PhysxRigidBodyAPI.Apply(pole).CreateAngularDampingAttr(0.0)
    return root


def read_properties(robot, model):
    """直接调用 PhysX tensor view getter，而非读取 robot.data 写入缓存。"""
    import numpy as np

    view = robot.root_view
    index = robot.backend_body_names.index("pole")
    mass = view.get_masses().numpy()[:, index]
    com = view.get_coms().numpy()[:, index]
    inertia = view.get_inertias().numpy()[:, index]
    i = model["pole_inertia_com"]
    np.testing.assert_allclose(mass, model["pole_mass_kg"], rtol=1e-5, atol=1e-7)
    np.testing.assert_allclose(com[:, :3], np.tile([0, 0, model["pole_com_m"]], (len(mass), 1)), atol=1e-6)
    np.testing.assert_allclose(inertia, np.tile([i, 0, 0, 0, i, 0, 0, 0, .01*i], (len(mass), 1)), rtol=1e-4, atol=1e-8)
    # COM frame rotation must be identity, allowing the equivalent opposite quaternion.
    np.testing.assert_allclose(com[:, 3:6], 0, atol=1e-6)
    np.testing.assert_allclose(abs(com[:, 6]), 1, atol=1e-6)
    return {"backend_body_names": list(robot.backend_body_names), "mass_kg": mass.tolist(),
            "com_pose_xyzw": com.tolist(), "inertia_com_kg_m2": inertia.tolist(), "passed": True}


def run(args):
    import numpy as np
    import torch
    import warp as wp
    wp.config.enable_backward = False
    from isaaclab.app import launch_simulation
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.sim.utils import clone
    from isaaclab.utils import instantiate
    from isaaclab_physx.physics import PhysxCfg
    from cartpole.device_profile import load_profile, model_parameters
    from cartpole.tasks.manager_based.cartpole.cartpole_env_cfg import CartpoleSceneCfg
    from cartpole.tasks.manager_based.cartpole.pole_spawner import spawn_v2_cartpole

    model = model_parameters(load_profile())
    cfg = CartpoleSceneCfg(num_envs=2, env_spacing=2.0)
    cfg.ground = None
    cfg.dome_light = None
    cfg.robot.actuators = {}
    cfg.robot.spawn.func = clone(spawn_fixed_cart) if args.mode == "swing" else spawn_v2_cartpole
    if args.mode == "swing":
        cfg.robot.init_state.joint_pos = {"cart_to_pole": math.pi + .05}
        cfg.robot.init_state.joint_vel = {"cart_to_pole": 0.0}
    sim_cfg = SimulationCfg(physics=PhysxCfg(), dt=.001, device=args.device or "cuda:0", gravity=(0, 0, -9.81))
    report = {"mode": args.mode, "num_envs": 2, "dt_s": .001, "scope": "独立仿真检查，不是策略或真机验收"}
    with launch_simulation(sim_cfg, args):
        sim = SimulationContext(sim_cfg)
        try:
            scene = instantiate(cfg)
            sim.reset()
            robot = scene["robot"]
            report["properties"] = read_properties(robot, model)
            if args.mode == "swing":
                # No policy/events/actuators: zero physical joint damping, friction and armature.
                robot.write_joint_stiffness_to_sim_index(stiffness=0.0)
                robot.write_joint_damping_to_sim_index(damping=0.0)
                robot.write_joint_armature_to_sim_index(armature=0.0)
                robot.write_joint_friction_coefficient_to_sim_index(
                    joint_friction_coeff=0.0, joint_dynamic_friction_coeff=0.0,
                    joint_viscous_friction_coeff=0.0)
                robot.write_joint_state_to_sim_index(
                    position=torch.full((2, 1), math.pi + .05, device=robot.device),
                    velocity=torch.zeros((2, 1), device=robot.device))
                scene.reset()
                view = robot.root_view
                if robot.backend_joint_names != ["cart_to_pole"]:
                    raise RuntimeError(f"固定小车后必须只剩杆转轴：{robot.backend_joint_names}")
                cart_index = robot.backend_body_names.index("cart")
                origin = view.get_link_transforms().numpy()[:, cart_index, :3].copy()
                times, angles = [], [[], []]
                drift = 0.0
                for step in range(round(args.seconds / sim_cfg.dt)):
                    sim.step(render=False)
                    scene.update(sim_cfg.dt)
                    q = view.get_dof_positions().numpy()[:, 0]
                    times.append((step + 1) * sim_cfg.dt)
                    for n in range(2):
                        angles[n].append(math.atan2(math.sin(float(q[n]) - math.pi), math.cos(float(q[n]) - math.pi)))
                    drift = max(drift, float(np.max(abs(view.get_link_transforms().numpy()[:, cart_index, :3] - origin))))
                expected = 2 * math.pi * math.sqrt(2 * model["pole_length_m"] / (3 * 9.81))
                report["swing"] = [measure_period(times, a, expected) for a in angles]
                report["cart_max_drift_m"] = drift
                report["max_amplitude_rad"] = max(abs(a) for values in angles for a in values)
                report["passed"] = all(r["passed"] for r in report["swing"]) and drift <= 1e-5 and report["max_amplitude_rad"] < .08
            else:
                report["passed"] = True
            print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
            if not report["passed"]:
                raise RuntimeError("物理验证未通过；请保留本次报告")
        finally:
            sim.stop()


def main():
    from isaaclab.app import add_launcher_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("properties", "swing"), default="properties")
    parser.add_argument("--seconds", type=float, default=8.0)
    add_launcher_args(parser)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 6 <= args.seconds <= 30:
        parser.error("--seconds 必须在 6 到 30 秒之间")
    if getattr(args, "physics", None) not in (None, "isaacsim_physx"):
        parser.error("本脚本只验证 Isaac Sim PhysX，不接受其他后端")
    run(args)


if __name__ == "__main__":
    main()
