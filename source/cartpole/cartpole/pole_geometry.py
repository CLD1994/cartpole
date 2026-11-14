"""在标准 Cartpole USD 上定义 V2 端点摆杆；不依赖 Isaac Lab 运行时。"""


def apply_pole_geometry(stage, root_path, model):
    """在环境克隆前替换摆杆几何，保持刚体及关节坐标不变。

    ``model`` 来自 device_profile.model_parameters。宽厚沿用原示意杆的
    1.6 × 3.6 mm，并非实测；轴向惯量沿用训练模型的横向惯量 1% 近似。
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    pole = stage.GetPrimAtPath(f"{root_path}/pole")
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{root_path}/cart/cart_to_pole"))
    children = [stage.GetPrimAtPath(f"{root_path}/pole/{name}") for name in ("visuals", "collisions")]
    if not pole or not joint or not all(children):
        raise ValueError("标准 Cartpole 资产缺少 pole、cart_to_pole 或摆杆几何子树")
    if list(joint.GetBody1Rel().GetTargets()) != [pole.GetPath()] or joint.GetLocalPos1Attr().Get() != Gf.Vec3f(0):
        raise ValueError("摆杆转轴必须连接 pole 且位于其局部原点")
    if joint.GetAxisAttr().Get() != "X" or joint.GetLocalRot1Attr().Get() != Gf.Quatf(1):
        raise ValueError("摆杆关节须绕局部 X 轴旋转且杆端坐标系不旋转")
    if pole.IsInstanceProxy():
        raise ValueError("pole 是实例代理，无法独立定义 V2 摆杆")

    # 关闭几何子树不会改变 pole 刚体及其外部关节关系。
    for child in children:
        child.SetActive(False)
    cube = UsdGeom.Cube.Define(stage, f"{root_path}/pole/rod")
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-.5), Gf.Vec3f(.5)])
    cube.CreateDisplayColorAttr([Gf.Vec3f(.8, .25, .1)])
    transform = UsdGeom.Xformable(cube)
    transform.ClearXformOpOrder()
    transform.AddTranslateOp().Set(Gf.Vec3d(0, 0, model["pole_length_m"] / 2))
    transform.AddScaleOp().Set(Gf.Vec3f(.0016, .0036, model["pole_length_m"]))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)

    mass = UsdPhysics.MassAPI.Apply(pole)
    mass.CreateMassAttr(model["pole_mass_kg"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, model["pole_com_m"]))
    inertia = model["pole_inertia_com"]
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia * .01))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    return cube.GetPrim()
