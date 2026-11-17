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
