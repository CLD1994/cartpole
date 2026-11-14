# V2 摆杆模型与验证

`Hexchip-Cartpole-Real-V2-v0` 使用标准 Cartpole 的小车、轨道和关节，加上项目定义的端点摆杆。场景创建时先替换杆，再复制到各环境。V1 不使用此替换。

## 尺寸与物理参数

摆杆长度来自 `device-profile.json`，为 0.49 m。`pole` 刚体局部原点就是转轴，杆沿 +Z 从 0 延伸到 0.49 m。视觉和碰撞共用一个长方体，宽厚为 1.6 × 3.6 mm，仅沿用示意模型的近似尺寸，并非实测。小车和轨道也只是示意外形。

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

程序直接读取 PhysX 关节角，用插值后的相邻正向过零时刻计算完整周期。成功需要至少三个完整周期，每个周期相对 `2π√(2L/(3g)) ≈ 1.147 s` 误差不超过 3%，小车位移不超过 10 µm，摆角保持在 0.08 rad 内。没有运动、采样不完整、参数不符都会报错或返回失败，不能把进程退出本身当作验收。

验收时同时检查退出码 0 和 JSON 中 `passed: true`。这些验证只证明指定理想条件下的仿真模型，不证明真实摩擦、驱动力、随机化分布或策略控制正确。之后恢复任务默认配置，再运行 64 环境、10 轮训练冒烟，保存新配置及 checkpoint，确认后才正式训练。

旧 checkpoint 保留作为对照，不将其表现视作新模型训练效果。外观修正不证明慢动作问题已完全解决；渲染速度、物理时间与控制器动态须分别观察。

## 验证状态

操作者已在 OpenUSD 0.26.8 执行几何检查，杆范围约为 z=−4.8e−9..0.4900000048 m，名义质量、质心和惯量通过。云端完成 Python 语法、差异格式检查及周期判定的四项纯单元测试。PhysX 属性读回、自由摆与新模型训练仍待在 Isaac Lab 3.0 环境执行。
