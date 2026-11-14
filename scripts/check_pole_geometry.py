"""只读检查 V2 摆杆创建规则；传入本机缓存的标准 cartpole.usd 路径。"""
import argparse
import json

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

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
    cart = stage.GetPrimAtPath("/cartpole/cart")
    old_render_paths = [prim.GetPath() for prim in Usd.PrimRange(cart, Usd.TraverseInstanceProxies()) if UsdGeom.Gprim(prim)]
    assert old_render_paths, "标准车体未找到实际渲染几何，不能确认外观替换"
    collision_states = {prim.GetPath(): UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                        for prim in Usd.PrimRange(cart, Usd.TraverseInstanceProxies()) if prim.HasAPI(UsdPhysics.CollisionAPI)}
    assert collision_states, "标准车体未找到碰撞体，不能确认物理保持"
    model = model_parameters(load_profile())
    apply_pole_geometry(stage, "/cartpole", model)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
    collision_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True)
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
        assert UsdGeom.Imageable(prim).ComputeVisibility() == "inherited", path
        assert not prim.HasAPI(UsdPhysics.CollisionAPI), path
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI), path
        assert not prim.HasAPI(UsdPhysics.MassAPI), path
    for path in old_render_paths:
        assert UsdGeom.Imageable(stage.GetPrimAtPath(path)).ComputeVisibility() == "invisible", path
    for path, enabled in collision_states.items():
        prim = stage.GetPrimAtPath(path)
        assert prim.IsActive() and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() == enabled, path
    material, _ = UsdShade.MaterialBindingAPI(visual).ComputeBoundMaterial()
    assert material and material.GetPath() == visual.GetPath().AppendChild("RodSurface"), "杆须使用指定深色表面"
    mass = UsdPhysics.MassAPI(stage.GetPrimAtPath("/cartpole/pole"))
    assert abs(mass.GetMassAttr().Get() - model["pole_mass_kg"]) < 1e-6
    assert abs(mass.GetCenterOfMassAttr().Get()[2] - model["pole_com_m"]) < 1e-6
    assert abs(mass.GetDiagonalInertiaAttr().Get()[0] - model["pole_inertia_com"]) < 1e-8
    print(json.dumps({"bounds_m": [list(low), list(high)], "mass_kg": mass.GetMassAttr().Get(),
                      "com_m": list(mass.GetCenterOfMassAttr().Get()),
                      "inertia_com_kg_m2": list(mass.GetDiagonalInertiaAttr().Get()),
                      "cart_collision_count": len(collision_states),
                      "hidden_cart_geometry": [str(path) for path in old_render_paths],
                      "scope": "OpenUSD 创建检查通过；不是物理后端或运动验证"}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
