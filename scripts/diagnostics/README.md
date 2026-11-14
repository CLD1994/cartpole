# 回放速度测量

这个工具只加载仿真策略，不连接真机，不训练、不导出模型、不修改 checkpoint。
输出 `PLAYBACK_TIMING` JSON 行；`rtf` 等于模拟时间除以墙钟时间，1 表示实时，0.2 表示模拟一秒需要现实五秒。
计时包含策略推理、物理步进、reset 和界面更新，不包含启动、资产加载、模型加载及预热。

## 对照方法

在同一 Python 环境、同一个仓库版本，用同一 checkpoint 和 seed 各启动一个新进程。不要同时运行两组。

```powershell
python .\scripts\diagnostics\playback_rtf.py --checkpoint <模型完整路径> --viz kit --info --seed 42
python .\scripts\diagnostics\playback_rtf.py --checkpoint <同一模型完整路径> --viz none --info --seed 42
```

默认预热500步，测量5000步，单环境。物理后端固定为 Isaac Sim PhysX；两组只改变是否启用界面。实际物理步长、控制周期、渲染间隔会随结果输出，不修改这些配置。

默认不 sleep，测量可达到的回放速度。追加 `--real-time` 才模拟官方 play 的补 sleep 节拍；慢于实时则不补 sleep，也不跳步追赶。两种模式必须分别比较。

固定种子用于提高可重复性，不承诺GPU逐位确定性。和官方play一致，加载当前任务配置而不是模型目录里的 env.yaml；play_mode关闭观测噪声，但保留reset随机化。

只有 KIT 组显著更慢时，才支持显示路径带来额外开销的判断；这仍不足以定位具体渲染或UI模块。两组都慢时应继续检查仿真、Python和GPU同步。速度比不能证明模型几何或动力学正确。

## 验证边界

计时单元通过注入时钟、步进和同步函数验证，无需Isaac Lab。GPU入口依照Isaac Lab 3.0公开接口编写，仍需在真实Isaac运行环境验证后使用其测量结果；单元测试不等于GPU验证。

```text
python -m unittest discover -s scripts/diagnostics -p "test_*.py"
```
