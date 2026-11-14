"""只读检查 V2 摆杆创建规则；传入本机缓存的标准 cartpole.usd 路径。"""
import argparse
import json

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from cartpole.device_profile import load_profile, model_parameters
from cartpole.pole_geometry import apply_pole_geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd", help="标准 Cartpole USD 路径")
    args = parser.parse_args()
    stage = Usd.Stage.CreateInMemory()
    root = stage.DefinePrim("/cartpole", "Xform")
    root.GetReferences().AddReference(args.usd)
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/cartpole/cart/cart_to_pole"))
    before = (joint.GetLocalPos0Attr().Get(), joint.GetLocalPos1Attr().Get(), joint.GetAxisAttr().Get())
    model = model_parameters(load_profile())
    apply_pole_geometry(stage, "/cartpole", model)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    collision_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], ignoreVisibility=True)
    bounds = collision_cache.ComputeRelativeBound(stage.GetPrimAtPath("/cartpole/pole/rod"), stage.GetPrimAtPath("/cartpole/pole")).ComputeAlignedRange()
    low, high = bounds.GetMin(), bounds.GetMax()
    assert abs(low[2]) < 1e-6 and abs(high[2] - model["pole_length_m"]) < 1e-6, (low, high)
    assert (joint.GetLocalPos0Attr().Get(), joint.GetLocalPos1Attr().Get(), joint.GetAxisAttr().Get()) == before
    assert not stage.GetPrimAtPath("/cartpole/pole/visuals").IsActive()
    assert not stage.GetPrimAtPath("/cartpole/pole/collisions").IsActive()
    rod = stage.GetPrimAtPath("/cartpole/pole/rod")
    assert UsdPhysics.CollisionAPI(rod).GetCollisionEnabledAttr().Get()
    assert UsdGeom.Imageable(rod).GetVisibilityAttr().Get() == "invisible"
    visual = stage.GetPrimAtPath("/cartpole/pole/rod_visual")
    visual_range = cache.ComputeRelativeBound(visual, stage.GetPrimAtPath("/cartpole/pole")).ComputeAlignedRange()
    assert abs(visual_range.GetMin()[2]) < 1e-6
    assert abs(visual_range.GetMax()[2] - model["pole_length_m"]) < 1e-6
    for path in ("pole/rod_visual", "cart/body_visual", "cart/hub_visual"):
        prim = stage.GetPrimAtPath(f"/cartpole/{path}")
        assert not prim.HasAPI(UsdPhysics.CollisionAPI), path
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI), path
        assert not prim.HasAPI(UsdPhysics.MassAPI), path
    assert UsdGeom.Imageable(stage.GetPrimAtPath("/cartpole/cart/visuals")).GetVisibilityAttr().Get() == "invisible"
    mass = UsdPhysics.MassAPI(stage.GetPrimAtPath("/cartpole/pole"))
    assert abs(mass.GetMassAttr().Get() - model["pole_mass_kg"]) < 1e-6
    assert abs(mass.GetCenterOfMassAttr().Get()[2] - model["pole_com_m"]) < 1e-6
    assert abs(mass.GetDiagonalInertiaAttr().Get()[0] - model["pole_inertia_com"]) < 1e-8
    print(json.dumps({"bounds_m": [list(low), list(high)], "mass_kg": mass.GetMassAttr().Get(),
                      "com_m": list(mass.GetCenterOfMassAttr().Get()),
                      "inertia_com_kg_m2": list(mass.GetDiagonalInertiaAttr().Get()),
                      "scope": "OpenUSD 创建检查通过；不是物理后端或运动验证"}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
