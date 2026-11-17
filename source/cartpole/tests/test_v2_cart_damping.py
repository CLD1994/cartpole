"""Verify the actual V2 reset-event literal without requiring a simulator."""
import ast
from pathlib import Path
import unittest

class CartDampingTests(unittest.TestCase):
    def test_v2_cart_range_and_unchanged_pole_range(self):
        path=Path(__file__).parents[1]/'cartpole/tasks/manager_based/cartpole/cartpole_env_cfg_sim2real_v2.py'
        tree=ast.parse(path.read_text(encoding='utf8'))
        call=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign)
            and any(ast.unparse(t)=='self.events.randomize_joint_damping' for t in n.targets))
        self.assertEqual(next(ast.literal_eval(k.value) for k in call.keywords if k.arg=='mode'),'reset')
        params=next(k.value for k in call.keywords if k.arg=='params')
        ranges=next(ast.literal_eval(v) for k,v in zip(params.keys,params.values)
            if ast.literal_eval(k)=='damping_range')
        self.assertEqual(ranges,{'slider_to_cart':(.1,.6),'cart_to_pole':(0.,.01)})

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
