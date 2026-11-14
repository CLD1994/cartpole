"""公开 PhysX view 边界替身：确认后端数据而非 setter 缓存决定验收。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location("pole_physics_readback", Path(__file__).parents[3] / "scripts/check_pole_physics.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class View:
    def __init__(self, friction):
        self.friction = friction

    def get_dof_stiffnesses(self):
        return SimpleNamespace(numpy=lambda: np.zeros((2, 1)))

    def get_dof_dampings(self):
        return SimpleNamespace(numpy=lambda: np.zeros((2, 1)))

    def get_dof_armatures(self):
        return SimpleNamespace(numpy=lambda: np.zeros((2, 1)))

    def get_dof_friction_properties(self):
        return SimpleNamespace(numpy=lambda: self.friction)


class FreeJointReadbackTests(unittest.TestCase):
    def test_zero_backend_properties_pass(self):
        result = probe.check_free_joint_properties(View(np.zeros((2, 1, 3))))
        self.assertEqual(result["friction_static_dynamic_viscous"], [[[0., 0., 0.]], [[0., 0., 0.]]])

    def test_each_nonzero_friction_component_fails(self):
        for column in range(3):
            values = np.zeros((2, 1, 3))
            values[1, 0, column] = .1
            with self.subTest(column=column), self.assertRaises(AssertionError):
                probe.check_free_joint_properties(View(values))


if __name__ == "__main__":
    unittest.main()
