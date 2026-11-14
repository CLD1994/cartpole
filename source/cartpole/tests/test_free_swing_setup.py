"""真实 Torch 数据与公开资产接口替身验证自由摆写入契约。"""
import importlib.util
import math
from pathlib import Path
import unittest
import torch

spec = importlib.util.spec_from_file_location("pole_physics_setup", Path(__file__).parents[3] / "scripts/check_pole_physics.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class Asset:
    device = "cpu"
    num_instances = 2
    num_joints = 1
    backend_joint_names = ["cart_to_pole"]

    def __init__(self):
        self.writes = {}

    def record(self, **fields):
        for name, value in fields.items():
            if not isinstance(value, torch.Tensor) or value.shape != (self.num_instances, self.num_joints):
                raise AssertionError(f"{name} 必须匹配环境×关节维度")
            if value.dtype != torch.float32 or str(value.device) != self.device or not value.is_contiguous():
                raise AssertionError(f"{name} 必须为目标设备上的连续float32")
            self.writes[name] = value.clone()

    def write_joint_stiffness_to_sim_index(self, *, stiffness):
        self.record(stiffness=stiffness)

    def write_joint_damping_to_sim_index(self, *, damping):
        self.record(damping=damping)

    def write_joint_armature_to_sim_index(self, *, armature):
        self.record(armature=armature)

    def write_joint_friction_coefficient_to_sim_index(self, *, joint_friction_coeff, joint_dynamic_friction_coeff, joint_viscous_friction_coeff):
        self.record(static=joint_friction_coeff, dynamic=joint_dynamic_friction_coeff, viscous=joint_viscous_friction_coeff)

    def write_joint_state_to_sim_index(self, *, position, velocity):
        self.record(position=position, velocity=velocity)


class FreeSwingSetupTests(unittest.TestCase):
    def test_all_parameters_use_explicit_float32_and_device(self):
        old = torch.get_default_dtype()
        try:
            torch.set_default_dtype(torch.float64)
            for count in (1, 2, 3):
                with self.subTest(num_instances=count):
                    robot = Asset()
                    robot.num_instances = count
                    probe.prepare_free_swing(robot)
                    self.assertEqual(set(robot.writes), {"stiffness", "damping", "armature", "static", "dynamic", "viscous", "position", "velocity"})
                    for name, value in robot.writes.items():
                        expected = math.pi + .05 if name == "position" else 0
                        torch.testing.assert_close(value, torch.full(value.shape, expected, dtype=torch.float32))
        finally:
            torch.set_default_dtype(old)

    def test_invalid_topology_is_rejected_before_any_write(self):
        robot = Asset()
        robot.backend_joint_names = ["slider_to_cart", "cart_to_pole"]
        robot.num_joints = 2
        with self.assertRaises(ValueError):
            probe.prepare_free_swing(robot)
        self.assertEqual(robot.writes, {})

    def test_empty_scene_is_rejected_before_any_write(self):
        robot = Asset()
        robot.num_instances = 0
        with self.assertRaises(ValueError):
            probe.prepare_free_swing(robot)
        self.assertEqual(robot.writes, {})


if __name__ == "__main__":
    unittest.main()
