# V2 摆杆模型与验证

`Hexchip-Cartpole-Real-V2-v0` 使用标准 Cartpole 的小车、轨道和关节，加上项目定义的端点摆杆。场景创建时先替换杆，再复制到各环境。V1 不使用此替换。

## 尺寸与物理参数

摆杆长度来自 `device-profile.json`，为 0.49 m。`pole` 刚体局部原点就是转轴，杆沿 +Z 从 0 延伸到 0.49 m。视觉使用直径 8 mm 的圆杆，碰撞保留宽厚 1.6 × 3.6 mm 的长方体并隐藏显示。小车视觉为 80 × 75 × 55 mm 的紧凑车体，侧面转轴直径 24 mm。上述宽径与车体尺寸均是便于观察的示意值，不是实测；轨道也保留示意外形。新增视觉几何没有碰撞、刚体或质量 API，不改变物理参数或关节位置。

名义质量 0.2316 kg，质心位于 `(0, 0, 0.245)` m。绕质心的横向惯量为 `m L² / 12 = 0.00463393 kg·m²`。转轴惯量为 `m L² / 3 = 0.01853572 kg·m²`，不将它写作质心惯量。轴向惯量取横向惯量的 1%，是正值近似，不是实测。

每次回合重置仍随机化质量 ±10%、质心位置 ±3%、横向惯量比例 0.85–1.15；杆长固定。这些随机化是训练假设，不代表每条杆都是严格的均匀细杆。日志中的 `application cache` 是应用侧属性缓存，不是独立物理读回。

## 几何检查

在安装了项目和 OpenUSD 的 Isaac Lab Python 环境运行，参数指向本机缓存的标准 USD：

```powershell
python .\scripts\check_pole_geometry.py "C:\完整路径\cartpole.usd"
```

此命令在内存创建场景，不修改缓存资产。它检查杆上下端、碰撞启用、原几何停用、关节锚点及名义质量/质心/惯量。通过仅说明 USD 定义正确，不能代替 PhysX 初始化后的验证。

## 物理后端检查

使用本项目同一个 Isaac Lab 3.0 解释器，两个命令分别启动并自动关闭独立场景。均创建两个环境，不加载策略、训练回合事件或串口，不读取、写入或删除 checkpoint。

```powershell
$env:PYTHONIOENCODING = "utf-8"
python .\scripts\check_pole_physics.py --mode properties --viz none --info *> .\pole-properties.log
"退出码: $LASTEXITCODE"
Get-Content .\pole-properties.log -Tail 100 -Encoding utf8
```

`properties` 保留 V2 关节拓扑，关闭执行器，不进行训练；直接从 Isaac Sim PhysX 的 `robot.root_view.get_masses()`、`get_coms()`、`get_inertias()` 读取两个环境的杆属性。按 `backend_body_names` 选择杆，不读取 `robot.data` 缓存。验证名义质量、质心位置和坐标旋转、完整惯量矩阵。此接口属于 `isaaclab_physx.PhysxCfg`，不能替换为另一后端的 `get_attribute`。

```powershell
python .\scripts\check_pole_physics.py --mode swing --seconds 8 --viz none --info *> .\pole-swing.log
"退出码: $LASTEXITCODE"
Get-Content .\pole-swing.log -Tail 150 -Encoding utf8
```

`swing` 仅在这次诊断场景中把小车滑动关节改成固定关节，关闭碰撞、驱动、杆体阻尼、关节摩擦、armature（附加关节惯量）、睡眠和稳定化。它不修改训练配置或缓存 USD。两个杆都从下垂方向偏离 0.05 rad、角速度零释放，物理步长 1 ms，重力 9.81 m/s²，默认运行 8 秒仿真时间。

所有关节写入使用目标设备上的二维 float32 Tensor，尺寸为环境数×关节数；开始前检查固定场景仅剩杆转轴。随后独立读回后端刚度、阻尼、附加惯量和三项摩擦，确认均为零。程序直接读取 PhysX 关节角，用插值后的相邻正向过零时刻计算完整周期。成功需要至少三个完整周期，每个周期相对 `2π√(2L/(3g)) ≈ 1.147 s` 误差不超过 3%，小车位移不超过 10 µm，摆角保持在 0.08 rad 内。没有运动、采样不完整、参数不符都会报错或返回失败，不能把进程退出本身当作验收。

验收时同时检查退出码 0 和 JSON 中 `passed: true`。这些验证只证明指定理想条件下的仿真模型，不证明真实摩擦、驱动力、随机化分布或策略控制正确。之后恢复任务默认配置，再运行 64 环境、10 轮训练冒烟，保存新配置及 checkpoint，确认后才正式训练。

旧 checkpoint 保留作为对照，不将其表现视作新模型训练效果。外观修正不证明慢动作问题已完全解决；渲染速度、物理时间与控制器动态须分别观察。

## 验证状态

操作者已在 OpenUSD 0.26.8 执行几何检查，杆范围约为 z=−4.8e−9..0.4900000048 m，名义质量、质心和惯量通过。云端完成 Python 语法、差异格式检查、周期判定的四项纯单元测试及两项配置/脚本导入边界测试。操作者已在 Isaac Lab 3.0 完成两个环境的名义属性后端读回：质量约 0.231600016 kg、质心 0.245000005 m、横向惯量 0.00463393 kg·m²。自由摆和新模型训练仍待完成。自由摆准备另有三项真实 Torch CPU 接口契约测试、两项 NumPy 后端读回判定测试；它们不替代 GPU/PhysX 运行。

## USD 与启动顺序

几何检查使用独立 Python 进程中的 OpenUSD。物理检查则由 `launch_simulation` 先启动 Kit，再导入场景执行模块和调用摆杆 spawner。配置加载只保留 spawner 函数引用，不应加载其 USD 执行依赖。不要在同一 Python 进程先运行几何检查或导入独立 `pxr` 再启动 Kit。

物理检查在启动前打印 `pre_launch_usd_modules`；正常应为空对象。如果已经发现 `pxr.Usd/Tf/Gf/Sdf`，脚本输出其文件路径并停止，不尝试卸载已加载 DLL 或修改系统路径。遇到 Windows 程序入口点/DLL 错误时，保留完整日志与解释器路径，先核查加载来源；不要直接卸载包或删除 Isaac Sim 缓存。
