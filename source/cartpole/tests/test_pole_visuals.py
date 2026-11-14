"""纯视觉作者函数的公开 USD 协作者契约，不替代 OpenUSD/PhysX 集成验证。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch, Mock

source = Path(__file__).parents[1] / 'cartpole/pole_geometry.py'
spec = importlib.util.spec_from_file_location('pole_geometry', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Attr:
    def __init__(self, value=None): self.value = value
    def Set(self, value): self.value = value
    def Get(self): return self.value


class Shape:
    def __init__(self): self.attrs = {}; self.children = []; self.imageable = True; self.instance = False
    def __getattr__(self, name):
        if name.startswith('Create') and name.endswith('Attr'):
            def create(value):
                self.attrs[name[6:-4]] = Attr(value)
                return self.attrs[name[6:-4]]
            return create
        raise AttributeError(name)
    def GetChildren(self): return self.children
    def GetPath(self): return Mock()
    def GetPrim(self): return self
    def IsInstanceProxy(self): return False
    def IsInstance(self): return self.instance
    def SetInstanceable(self, value): self.instance = value
    def ClearXformOpOrder(self): pass
    def AddTranslateOp(self): self.attrs['Translate'] = Attr(); return self.attrs['Translate']
    def AddOrientOp(self): self.attrs["Orient"] = Attr(); return self.attrs["Orient"]
    def GetAxisAttr(self): return Attr("X")
    def GetLocalRot0Attr(self): return Attr((.70710678, 0, 0, .70710678))
    def AddScaleOp(self): self.attrs['Scale'] = Attr(); return self.attrs['Scale']
    def GetLocalPos0Attr(self): return Attr((.12, 0, 0))


class Stage:
    def __init__(self):
        self.prims = {f'/r/cart{suffix}': Shape() for suffix in ('', '/visuals', '/collisions', '/cart_to_pole')}
        self.prims.update({'/r/slider': Shape(), '/r/slider/slider_to_cart': Shape()})
        self.prims['/r/cart'].children = [self.prims['/r/cart/visuals'], self.prims['/r/cart/collisions']]
    def GetPrimAtPath(self, path): return self.prims.get(path)
    def define(self, path):
        self.prims[path] = Shape()
        return self.prims[path]


class Vector(tuple):
    def __sub__(self, other): return Vector(a-b for a,b in zip(self, other))
    def GetLength(self): return sum(x*x for x in self) ** .5

def vector(*args): return Vector(args if len(args) > 1 else args * 3)

class Matrix:
    def __mul__(self, other): return self
    def GetInverse(self): return self
    def Transform(self, value): return value
    def TransformDir(self, value): return value



class VisualTests(unittest.TestCase):
    def setUp(self):
        self.stage = Stage()
        factory = SimpleNamespace(Define=lambda stage, path: stage.define(path))
        self.usd = SimpleNamespace(Gf=SimpleNamespace(Vec3f=vector, Vec3d=vector),
            UsdGeom=SimpleNamespace(Cube=factory, Cylinder=factory, Xformable=lambda p:p,
                XformCache=lambda: SimpleNamespace(GetLocalToWorldTransform=lambda p: Matrix()),
                Imageable=lambda p:p if p.imageable else None, Tokens=SimpleNamespace(invisible='invisible', inherited='inherited')),
            # No mutation APIs: any attempt to create collision/mass/rigid bodies fails.
            UsdPhysics=SimpleNamespace(RevoluteJoint=lambda p:p, PrismaticJoint=lambda p:p), Sdf=Mock(), UsdShade=Mock())
    def apply(self):
        with patch.dict(sys.modules, {'pxr': self.usd}):
            module.apply_cartpole_visuals(self.stage, '/r', {'pole_length_m': .49})
    def test_rod_has_correct_endpoints_and_visual_radius(self):
        self.apply()
        attrs = self.stage.prims['/r/pole/rod_visual'].attrs
        self.assertEqual(attrs['Height'].value, .49)
        self.assertEqual(attrs['Radius'].value, .004)
        self.assertEqual(attrs['DisplayColor'].value, [(.015, .16, .75)])
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
    def test_collision_rendering_hidden_without_removing_physics(self):
        collision = self.stage.prims['/r/cart/collisions']
        collision.attrs['CollisionEnabled'] = Attr(True)
        self.apply()
        self.assertIs(self.stage.prims['/r/cart/collisions'], collision)
        self.assertTrue(collision.attrs['CollisionEnabled'].value)
        self.assertEqual(collision.attrs['Visibility'].value, 'invisible')
        for path in ('/r/cart/body_visual', '/r/cart/hub_visual', '/r/pole/rod_visual'):
            self.assertEqual(self.stage.prims[path].attrs['Visibility'].value, 'inherited')

    def test_non_imageable_container_descends_to_real_geometry(self):
        scope = self.stage.prims['/r/cart/visuals']
        scope.imageable = False
        mesh = Shape()
        scope.children = [mesh]
        self.apply()
        self.assertNotIn('Visibility', scope.attrs)
        self.assertEqual(mesh.attrs['Visibility'].value, 'invisible')

    def test_non_imageable_instance_container_is_expanded_before_children(self):
        scope = self.stage.prims['/r/cart/visuals']
        scope.imageable = False
        scope.instance = True
        mesh = Shape()
        mesh.IsInstanceProxy = lambda: scope.instance
        scope.children = [mesh]
        self.apply()
        self.assertFalse(scope.instance)
        self.assertEqual(mesh.attrs['Visibility'].value, 'invisible')

    def test_static_rail_touches_smaller_cart_bottom(self):
        self.apply()
        body = self.stage.prims['/r/cart/body_visual'].attrs
        rail = self.stage.prims['/r/slider/rail_visual'].attrs
        self.assertEqual(body['Scale'].value, (.04, .05, .024))
        self.assertEqual(rail['Scale'].value, (.9, .04, .04))
        self.assertAlmostEqual(body['Translate'].value[2] - .012, rail['Translate'].value[2] + .02)
        self.assertEqual(body['Translate'].value[0], rail['Translate'].value[0])

    def test_missing_cart_visual_fails_before_editing(self):
        del self.stage.prims['/r/cart/visuals']
        with self.assertRaises(ValueError): self.apply()
        self.assertEqual(set(self.stage.prims), {'/r/cart', '/r/cart/collisions', '/r/cart/cart_to_pole', '/r/slider', '/r/slider/slider_to_cart'})

if __name__ == '__main__': unittest.main()
