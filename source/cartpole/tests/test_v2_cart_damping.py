"""Check V2 configuration expressions without requiring a simulator."""
import ast
from pathlib import Path
import unittest

class CartDampingTests(unittest.TestCase):
    def test_v2_reset_events_use_device_profile_ranges(self):
        path = Path(__file__).parents[1] / 'cartpole/tasks/manager_based/cartpole/cartpole_env_cfg_sim2real_v2.py'
        tree = ast.parse(path.read_text(encoding='utf8'))
        device = {
            'cart_mass_range_kg': (.046, .086),
            'pole_mass_range_kg': (.19, .27),
            'pole_mass_kg': .23,
            'pole_length_m': .49,
            'pole_com_m': .245,
            'pole_com_offset_range_m': (.22, .28),
            'pole_inertia_scale_range': (.8, 1.2),
            'cart_damping_range_n_s_per_m': (.2, .4),
            'pole_damping_range_n_m_s_per_rad': (.003, .007),
            'cart_mass_kg': .066,
            'cart_mass_randomization_fraction': .1,
        }
        expected = {
            'randomize_body_mass': {'mass_range': {'base_link|cart': (.046, .086)}},
            'randomize_pole_properties': {
                'mass_range': (.19, .27), 'length': .49, 'com_offset_range': (.22, .28),
                'inertia_scale_range': (.8, 1.2),
            },
            'randomize_joint_damping': {
                'damping_range': {'slider_to_cart': (.2, .4), 'cart_to_pole': (.003, .007)},
            },
        }
        for event, expected_params in expected.items():
            with self.subTest(event=event):
                call = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                            and any(ast.unparse(t) == f'self.events.{event}' for t in n.targets))
                self.assertEqual(next(ast.literal_eval(k.value) for k in call.keywords
                                      if k.arg == 'mode'), 'reset')
                params = next(k.value for k in call.keywords if k.arg == 'params')
                actual = {
                    key: eval(compile(ast.Expression(v), '<V2 reset event>', 'eval'),
                              {'__builtins__': {}, 'DEVICE': device})
                    for k, v in zip(params.keys, params.values)
                    if (key := ast.literal_eval(k)) in expected_params
                }
                self.assertEqual(actual, expected_params)

    def test_v2_training_failure_angle_is_twenty_degrees(self):
        import math
        path=Path(__file__).parents[1]/'cartpole/tasks/manager_based/cartpole/cartpole_env_cfg_sim2real_v2.py'
        tree=ast.parse(path.read_text(encoding='utf8'))
        call=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign)
            and any(ast.unparse(t)=='self.terminations.pole_out_of_bounds' for t in n.targets))
        params=next(k.value for k in call.keywords if k.arg=='params')
        expr=next(v for k,v in zip(params.keys,params.values) if ast.literal_eval(k)=='max_angle')
        value=eval(compile(ast.Expression(expr),'<training angle>','eval'),{'math':math})
        self.assertAlmostEqual(value,math.radians(20))

    def test_real_termination_twenty_degree_signed_boundaries(self):
        import importlib.util
        import math
        import torch
        from types import SimpleNamespace as NS
        path=Path(__file__).parents[1]/'cartpole/tasks/manager_based/cartpole/mdp/terminations.py'
        spec=importlib.util.spec_from_file_location('training_termination',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        angles=torch.tensor([[math.radians(d)] for d in (-20.001,-20.,-19.999,0.,19.999,20.,20.001)],dtype=torch.float64)
        env=NS(scene={'robot':NS(data=NS(joint_pos=NS(torch=angles)))})
        actual=module.pole_angle_out_of_bounds(env,NS(name='robot',joint_ids=[0]),math.radians(20.))
        self.assertEqual(actual.tolist(),[True,False,False,False,False,False,True])
