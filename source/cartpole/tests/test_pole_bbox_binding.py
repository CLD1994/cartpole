"""检查脚本使用位置参数调用 OpenUSD BBoxCache 的绑定契约。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch


class ConstructorReached(Exception):
    pass


class BBoxBindingTests(unittest.TestCase):
    def test_geometry_main_accepts_positional_only_bbox_constructor(self):
        calls = []
        def bbox(time, purposes, use_extents_hint=False, ignore_visibility=False, /):
            calls.append((use_extents_hint, ignore_visibility))
            if ignore_visibility:
                raise ConstructorReached
            return Mock()
        usd = SimpleNamespace(Usd=Mock(), Gf=Mock(), UsdPhysics=Mock(),
                              UsdGeom=SimpleNamespace(BBoxCache=bbox))
        profile = SimpleNamespace(load_profile=lambda: {}, model_parameters=lambda _: {})
        geometry = SimpleNamespace(apply_pole_geometry=lambda *args: None)
        modules = {'pxr': usd, 'cartpole': Mock(), 'cartpole.device_profile': profile,
                   'cartpole.pole_geometry': geometry}
        source = Path(__file__).parents[3] / 'scripts/check_pole_geometry.py'
        spec = importlib.util.spec_from_file_location('geometry_binding_check', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch.object(sys, 'argv', ['check_pole_geometry.py', 'fixture.usd']):
            spec.loader.exec_module(module)
            with self.assertRaises(ConstructorReached):
                module.main()
        self.assertEqual(calls, [(False, False), (False, True)])


if __name__ == '__main__': unittest.main()
