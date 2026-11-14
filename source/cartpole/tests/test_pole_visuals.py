"""纯视觉作者函数的公开 USD 协作者契约，不替代 OpenUSD/PhysX 集成验证。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

source = Path(__file__).parents[1] / 'cartpole/pole_geometry.py'
spec = importlib.util.spec_from_file_location('pole_geometry', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Attr:
    def __init__(self, value=None): self.value = value
    def Set(self, value): self.value = value
    def Get(self): return self.value


class Shape:
    def __init__(self): self.attrs = {}
    def __getattr__(self, name):
        if name.startswith('Create') and name.endswith('Attr'):
            def create(value):
                self.attrs[name[6:-4]] = Attr(value)
                return self.attrs[name[6:-4]]
            return create
        raise AttributeError(name)
    def IsInstanceProxy(self): return False
    def ClearXformOpOrder(self): pass
    def AddTranslateOp(self): self.attrs['Translate'] = Attr(); return self.attrs['Translate']
    def AddScaleOp(self): self.attrs['Scale'] = Attr(); return self.attrs['Scale']
    def GetLocalPos0Attr(self): return Attr((.12, 0, 0))


class Stage:
    def __init__(self):
        self.prims = {f'/r/cart{suffix}': Shape() for suffix in ('', '/visuals', '/cart_to_pole')}
    def GetPrimAtPath(self, path): return self.prims.get(path)
    def define(self, path):
        self.prims[path] = Shape()
        return self.prims[path]


def vector(*args): return args if len(args) > 1 else args * 3


class VisualTests(unittest.TestCase):
    def setUp(self):
        self.stage = Stage()
        factory = SimpleNamespace(Define=lambda stage, path: stage.define(path))
        self.usd = SimpleNamespace(Gf=SimpleNamespace(Vec3f=vector, Vec3d=vector),
            UsdGeom=SimpleNamespace(Cube=factory, Cylinder=factory, Xformable=lambda p:p,
                Imageable=lambda p:p, Tokens=SimpleNamespace(invisible='invisible')),
            # No mutation APIs: any attempt to create collision/mass/rigid bodies fails.
            UsdPhysics=SimpleNamespace(RevoluteJoint=lambda p:p))
    def apply(self):
        with patch.dict(sys.modules, {'pxr': self.usd}):
            module.apply_cartpole_visuals(self.stage, '/r', {'pole_length_m': .49})
    def test_rod_has_correct_endpoints_and_visual_radius(self):
        self.apply()
        attrs = self.stage.prims['/r/pole/rod_visual'].attrs
        self.assertEqual(attrs['Height'].value, .49)
        self.assertEqual(attrs['Radius'].value, .004)
        self.assertEqual(attrs['Translate'].value, (0, 0, .245))
        self.assertEqual(attrs['Extent'].value, [(-.004, -.004, -.245), (.004, .004, .245)])
    def test_cart_hub_ends_at_unchanged_joint_anchor(self):
        joint = self.stage.prims['/r/cart/cart_to_pole']
        self.apply()
        self.assertIs(self.stage.prims['/r/cart/cart_to_pole'], joint)
        hub = self.stage.prims['/r/cart/hub_visual'].attrs
        self.assertEqual(hub['Axis'].value, 'X')
        self.assertAlmostEqual(hub['Translate'].value[0] + hub['Height'].value / 2, .12)
        self.assertEqual(self.stage.prims['/r/cart/visuals'].attrs['Visibility'].value, 'invisible')
    def test_missing_cart_visual_fails_before_editing(self):
        del self.stage.prims['/r/cart/visuals']
        with self.assertRaises(ValueError): self.apply()
        self.assertEqual(set(self.stage.prims), {'/r/cart', '/r/cart/cart_to_pole'})

if __name__ == '__main__': unittest.main()
