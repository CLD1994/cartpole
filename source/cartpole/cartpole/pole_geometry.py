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

    cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    apply_cartpole_visuals(stage, root_path, model)

    mass = UsdPhysics.MassAPI.Apply(pole)
    mass.CreateMassAttr(model["pole_mass_kg"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, model["pole_com_m"]))
    inertia = model["pole_inertia_com"]
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia * .01))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    return cube.GetPrim()


def apply_cartpole_visuals(stage, root_path, model):
    """添加纯视觉示意，不修改碰撞、刚体或关节；宽径并非实测尺寸。"""
    from pxr import Gf, UsdGeom, UsdPhysics

    cart = stage.GetPrimAtPath(f"{root_path}/cart")
    original = stage.GetPrimAtPath(f"{root_path}/cart/visuals")
    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f"{root_path}/cart/cart_to_pole"))
    if not cart or not original or not joint:
        raise ValueError("标准 Cartpole 缺少车体视觉或转轴，无法创建示意外观")
    if cart.IsInstanceProxy() or original.IsInstanceProxy():
        raise ValueError("车体视觉是实例代理，不能独立定义示意外观")
    # 容器可能不是 Imageable；沿真实层级找到可隐藏的几何子树。
    for child in cart.GetChildren():
        hide_render_subtree(child)
    anchor = joint.GetLocalPos0Attr().Get()

    rod = UsdGeom.Cylinder.Define(stage, f"{root_path}/pole/rod_visual")
    rod.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    rod.CreateAxisAttr("Z")
    rod.CreateRadiusAttr(.004)
    rod.CreateHeightAttr(model["pole_length_m"])
    half_length = model["pole_length_m"] / 2
    rod.CreateExtentAttr([Gf.Vec3f(-.004, -.004, -half_length), Gf.Vec3f(.004, .004, half_length)])
    rod.CreateDisplayColorAttr([Gf.Vec3f(.015, .16, .75)])
    apply_rod_material(stage, rod)
    xform = UsdGeom.Xformable(rod)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, model["pole_length_m"] / 2))

    body = UsdGeom.Cube.Define(stage, f"{root_path}/cart/body_visual")
    body.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    body.CreateSizeAttr(1.)
    body.CreateExtentAttr([Gf.Vec3f(-.5), Gf.Vec3f(.5)])
    body.CreateDisplayColorAttr([Gf.Vec3f(.12, .17, .22)])
    xform = UsdGeom.Xformable(body)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(anchor[0] - .025, anchor[1], anchor[2] - .012))
    xform.AddScaleOp().Set(Gf.Vec3f(.04, .05, .024))

    hub = UsdGeom.Cylinder.Define(stage, f"{root_path}/cart/hub_visual")
    hub.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    hub.CreateAxisAttr("X")
    hub.CreateRadiusAttr(.008)
    hub.CreateHeightAttr(.015)
    hub.CreateExtentAttr([Gf.Vec3f(-.0075, -.008, -.008), Gf.Vec3f(.0075, .008, .008)])
    hub.CreateDisplayColorAttr([Gf.Vec3f(.65, .68, .7)])
    xform = UsdGeom.Xformable(hub)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(anchor[0] - .0075, anchor[1], anchor[2]))

    apply_rail_visual(stage, root_path, Gf.Vec3d(anchor[0] - .025, anchor[1], anchor[2] - .012))

def apply_rod_material(stage, rod):
    """为示意杆绑定浅背景下易辨识的蓝色表面；只影响渲染，不创建物理材质。"""
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, rod.GetPath().AppendChild("RodSurface"))
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.015, .16, .75))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.65)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(rod.GetPrim()).Bind(material)


def hide_render_subtree(prim):
    """隐藏渲染而保留物理；非渲染容器继续遍历，实例仅在必要时展开。"""
    from pxr import UsdGeom

    if prim.IsInstanceProxy():
        raise ValueError(f"不能写入实例代理的可见性：{prim.GetPath()}")
    imageable = UsdGeom.Imageable(prim)
    if imageable:
        imageable.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        return
    # 非 Imageable 实例根不能承载有效继承可见性，也不能写入其代理孩子。
    # 仅展开此几何子树的引用，不删除或停用任何碰撞、关节、刚体。
    if prim.IsInstance():
        prim.SetInstanceable(False)
    for child in prim.GetChildren():
        hide_render_subtree(child)


def apply_rail_visual(stage, root_path, cart_center):
    """静态轨道纯视觉：1000 mm 长，与车体底面相接，不改变滑动关节。"""
    from pxr import Gf, UsdGeom, UsdPhysics

    slider = stage.GetPrimAtPath(f"{root_path}/slider")
    cart = stage.GetPrimAtPath(f"{root_path}/cart")
    joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(f"{root_path}/slider/slider_to_cart"))
    if not slider or slider.IsInstanceProxy() or not joint or joint.GetAxisAttr().Get() != "X":
        raise ValueError("轨道示意需要标准 slider 和 X 轴滑动关节")
    for child in slider.GetChildren():
        hide_render_subtree(child)
    cache = UsdGeom.XformCache()
    cart_to_slider = cache.GetLocalToWorldTransform(cart) * cache.GetLocalToWorldTransform(slider).GetInverse()
    center = cart_to_slider.Transform(cart_center)
    vertical = cart_to_slider.TransformDir(Gf.Vec3d(0, 0, 1))
    if (vertical - Gf.Vec3d(0, 0, 1)).GetLength() > 1e-6:
        raise ValueError("车体与轨道竖直轴不一致，不能定位示意轨道")
    rail = UsdGeom.Cube.Define(stage, f"{root_path}/slider/rail_visual")
    rail.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
    rail.CreateSizeAttr(1.)
    rail.CreateExtentAttr([Gf.Vec3f(-.5), Gf.Vec3f(.5)])
    rail.CreateDisplayColorAttr([Gf.Vec3f(.75, .5, .08)])
    xform = UsdGeom.Xformable(rail)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2] - .012 - .02))
    xform.AddOrientOp().Set(joint.GetLocalRot0Attr().Get())
    xform.AddScaleOp().Set(Gf.Vec3f(1., .04, .04))
