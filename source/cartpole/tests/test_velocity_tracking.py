"""速度动作的单元测试：真实张量，隔离 Isaac Lab 场景与关节写入。"""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import torch


class JointEffortActionStub:
    """受 JointEffortAction 公共接口约束的场景替身，不模拟被测斜坡。"""

    def __init__(self, cfg, env):
        self.cfg, self._env = cfg, env
        self.device, self.num_envs = "cpu", 2
        self.processed_actions = torch.zeros((2, 1))
        self._joint_ids = [0]
        self._asset = SimpleNamespace(
            data=SimpleNamespace(joint_vel=torch.zeros((2, 1))),
            set_joint_effort_target_index=lambda **kwargs: None,
        )

    def process_actions(self, actions):
        self.processed_actions = actions.clone()

    def reset(self, env_ids=None):
        self.processed_actions[env_ids] = 0


def load_action():
    source = Path(__file__).resolve().parents[3] / "source/cartpole/cartpole/tasks/manager_based/cartpole/mdp/vel_tracking_action.py"
    joint_module = ModuleType("isaaclab.envs.mdp.actions.joint_actions")
    joint_module.JointEffortAction = JointEffortActionStub
    cfg_module = ModuleType("velocity_unit.vel_tracking_actions")
    cfg_module.JointVelocityTrackingActionCfg = object
    spec = importlib.util.spec_from_file_location("velocity_unit.vel_tracking_action", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {joint_module.__name__: joint_module, cfg_module.__name__: cfg_module}):
        spec.loader.exec_module(module)
    return module.JointVelocityTrackingAction


Action = load_action()


def make_action(decimation=2, delay=0, handles_decimation=False):
    cfg = SimpleNamespace(vel_max=1.0, tau=0.0, acc_max=5.0,
                          delay_steps=delay, max_delay_steps=3,
                          servo_gain=40.0, effort_limit=3.0)
    env = SimpleNamespace(step_dt=0.005, physics_dt=0.005 / decimation,
                          sim=SimpleNamespace(physics_manager=SimpleNamespace(
                              handles_decimation=lambda: handles_decimation)))
    return Action(cfg, env)


class VelocityTrackingTest(unittest.TestCase):
    def test_rejects_backend_that_owns_decimation(self):
        with self.assertRaises(ValueError):
            make_action(handles_decimation=True)

    def test_acceleration_is_independent_of_decimation(self):
        for decimation in (1, 2, 4):
            with self.subTest(decimation=decimation):
                action = make_action(decimation)
                action.process_actions(torch.ones((2, 1)))
                for _ in range(decimation):
                    action.apply_actions()
                torch.testing.assert_close(action._vel_ref, torch.full((2, 1), 0.025))

    def test_one_policy_step_delay_holds_target_across_substeps(self):
        action = make_action(delay=1)
        action.process_actions(torch.ones((2, 1)))
        action.apply_actions()
        action.apply_actions()
        torch.testing.assert_close(action._vel_ref, torch.zeros((2, 1)))
        action.process_actions(torch.zeros((2, 1)))
        action.apply_actions()
        action.apply_actions()
        torch.testing.assert_close(action._vel_ref, torch.full((2, 1), 0.025))

    def test_partial_reset_clears_only_selected_environment(self):
        action = make_action(delay=1)
        action.process_actions(torch.ones((2, 1)))
        action.apply_actions()
        action.apply_actions()
        action.reset([0])
        action.process_actions(torch.zeros((2, 1)))
        action.apply_actions()
        action.apply_actions()
        torch.testing.assert_close(action._vel_ref, torch.tensor([[0.0], [0.025]]))

    def test_zero_delay_uses_current_policy_action(self):
        action = make_action()
        action.process_actions(torch.ones((2, 1)))
        action.apply_actions()
        torch.testing.assert_close(action._vel_ref, torch.full((2, 1), 0.0125))


if __name__ == "__main__":
    unittest.main()
