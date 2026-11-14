"""以真实 CPU Tensor 和注入的公开环境/资产接口验证事件选择器契约。"""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import mock_open, patch

import torch

spec = spec_from_file_location(
    "cartpole_randomization_events",
    Path(__file__).parents[1] / "cartpole/tasks/manager_based/cartpole/mdp/dr_events.py",
)
events = module_from_spec(spec)
spec.loader.exec_module(events)


class Asset:
    def __init__(self):
        self.calls = []
        self.body_names = ["cart", "pole"]
        self.data = SimpleNamespace(
            body_mass=SimpleNamespace(torch=torch.ones(5, 2)),
            body_com_pose_b=torch.zeros(5, 2, 7),
            body_inertia=torch.zeros(5, 2, 9),
        )

    def find_bodies(self, name):
        return ([1], ["pole"]) if name == "pole" else ([0], ["cart"])

    def find_joints(self, name):
        return [0], [name]

    def set_masses_index(self, *, masses, body_ids, env_ids):
        self.calls.append((env_ids.clone(), masses.clone()))

    def set_coms_index(self, *, coms, body_ids, env_ids):
        self.calls.append((env_ids.clone(), coms.clone()))

    def set_inertias_index(self, *, inertias, body_ids, env_ids):
        self.calls.append((env_ids.clone(), inertias.clone()))

    def write_joint_damping_to_sim_index(self, *, damping, joint_ids, env_ids):
        self.calls.append((env_ids.clone(), damping.clone()))


def environment():
    asset = Asset()
    term = SimpleNamespace(
        tau=torch.zeros(5), vel_max=torch.zeros(5), acc_max=torch.zeros(5),
        delay_steps=torch.zeros(5, dtype=torch.long), max_delay_steps=8,
    )
    env = SimpleNamespace(
        num_envs=5, device="cpu", scene={"robot": asset},
        action_manager=SimpleNamespace(get_term=lambda name: term),
    )
    return env, asset, term


class RandomizationEventsTests(unittest.TestCase):
    def selectors(self):
        return [
            (None, [0, 1, 2, 3, 4]),
            (slice(None), [0, 1, 2, 3, 4]),
            (slice(1, 4), [1, 2, 3]),
            (slice(0, 5, 2), [0, 2, 4]),
            (slice(4, 0, -2), [4, 2]),
            (torch.tensor([3, 1]), [3, 1]),
            (torch.tensor([2], dtype=torch.int32), [2]),
            (slice(2, 2), []),
            (torch.tensor([], dtype=torch.long), []),
        ]

    def test_asset_events_write_only_selected_rows(self):
        cfg = SimpleNamespace(name="robot")
        cases = [
            (events.randomize_body_mass, dict(mass_range={"cart": (0.1, 0.1)}), 1),
            (events.randomize_joint_damping, dict(damping_range={"slider": (1., 1.)}), 1),
            (events.randomize_pole_properties, dict(mass_range=(0.2, 0.2), length=0.49), 3),
            (events.randomize_pole_axis_properties, dict(
                mass_range=(0.2, 0.2), length=0.49,
                com_offset_range=(0.245, 0.245), inertia_scale_range=(1., 1.)), 3),
        ]
        for function, kwargs, writes in cases:
            for selector, selected in self.selectors():
                with self.subTest(event=function.__name__, selected=selected, selector=repr(selector)):
                    env, asset, _ = environment()
                    # 日志输出也是外部依赖，替换公开文件接口，单测不读写磁盘。
                    with patch("builtins.open", mock_open()):
                        function(env, selector, cfg, **kwargs)
                    self.assertEqual(len(asset.calls), writes if selected else 0)
                    for ids, values in asset.calls:
                        self.assertEqual(ids.tolist(), selected)
                        self.assertEqual(values.shape[0], len(selected))
                        self.assertTrue(torch.isfinite(values).all())

    def test_action_events_preserve_unselected_environments(self):
        cases = [
            (events.randomize_motor_lag, dict(tau_range=(0.02, 0.02)), {"tau": 0.02}),
            (events.randomize_motor_delay, dict(delay_range=(3, 3)), {"delay_steps": 3}),
            (events.randomize_velocity_tracking, dict(
                vel_max_range=(1.25, 1.25), tau_range=(0.02, 0.02), acc_max_range=(10., 10.)),
             {"vel_max": 1.25, "tau": 0.02, "acc_max": 10.}),
        ]
        for function, kwargs, fields in cases:
            for selector, selected in self.selectors():
                with self.subTest(event=function.__name__, selected=selected, selector=repr(selector)):
                    env, _, term = environment()
                    function(env, selector, "joint_effort", **kwargs)
                    for name, value in fields.items():
                        actual = getattr(term, name)
                        expected = torch.zeros_like(actual)
                        expected[selected] = value
                        torch.testing.assert_close(actual, expected)


if __name__ == "__main__":
    unittest.main()
