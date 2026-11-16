# 倒立摆训练与部署

本项目基于 Isaac Lab，提供倒立摆仿真、PPO 训练、检查点筛选、模型导出和上位机部署工具。项目独立于 Isaac Lab 安装目录；ESP32 固件维护在 [device-nerve-esp32](https://github.com/CLD1994/device-nerve-esp32)。

## 从哪里开始

| 目标 | 文档入口 |
|---|---|
| 安装环境、训练、回放和导出 | [Isaac Lab 3.0 仿真工作流](deploy/v2/isaaclab-3-workflow.md) |
| 理解训练、动态候选和冻结评估的关系 | [训练与筛选入口](docs/training-deployment-workflow.md) |
| 使用 400 Hz、9 m/s² 的单动作配方 | [400 Hz / Acc9 配方](docs/training-400hz-acc9.md) |
| 了解完整回合与成功率的统计口径 | [训练指标](docs/training-episode-metrics.md) |
| 筛选检查点并进行独立 holdout | [动态候选](scripts/evaluation/TRAINING_CANDIDATES.md)、[评估工具](scripts/evaluation/README.md) |
| 连接现有 ESP32 ASCII 固件 | [设备部署与诊断](deploy/v2/device/README.md) |
| 核对设备档案和训练假设 | [设备档案与训练准备](deploy/v2/training-readiness.md) |
| 核对摆杆几何、惯量与验证边界 | [V2 摆杆模型](docs/v2-pole-model.md) |
| 分析仿真回放速度 | [性能诊断](scripts/diagnostics/README.md) |

## 当前主线范围

- 原始 V2 任务保留固定 500 Hz 基线；Window16、Tau0 和 Acc9-Hz400 是独立任务，不通过修改旧任务含义兼容新模型。
- 模型输出单一目标速度动作。400 Hz 配方使用 0.5 ms 物理步长、每 5 个物理步决策一次，参考加速度上限为 9 m/s²。它不是可变周期训练。
- 默认训练候选数量为 3；默认筛选已保存的动态候选，随后对胜者执行独立 holdout。训练指标用于提名，不等于冻结策略评估。
- `deploy/v2/device/` 提供当前 ASCII 固件的单动作运行器、会话管理及 STEP_LOSS 事件处理。控制节拍、固件状态上报频率和仿真物理步长是不同概念。
- 双动作训练、VELA 协议部署和自动低速探针仍是独立实验内容，不属于此主线流程。

`deploy/` 中还保留 V1 力控制原型，`deploy/v2/edge/` 和 `deploy/v2/pc_runner.py` 保留旧 V2 部署链路。当前协议入口见 [部署通信协议](deploy/v2/protocol.md)。不要把它们的固件、字段定义或操作命令与 `deploy/v2/device/` 的 ASCII 路径混用。

## 环境与安装

训练与可视化使用已安装 Isaac Lab 3.0 的 Python 环境。官方版本范围、启动参数和 Windows PowerShell 示例以[仿真工作流](deploy/v2/isaaclab-3-workflow.md)为准；无交互窗口使用 `--viz none`，不要对该训练入口使用旧的 `--headless` 参数。

将项目放在 Isaac Lab 目录之外，并在对应 Python 环境中安装：

```powershell
python -m pip install -e .\source\cartpole
python .\scripts\rsl_rl\train.py --help
python .\scripts\rsl_rl\play.py --help
```

训练、评估和导出必须使用同一任务及兼容配置。权重能加载、文件名相同或进程退出码为 0，都不能代替动作/观测顺序、单位、控制频率和模型文件的核验。

## 验证边界

已有现场记录包含后续实验分支的训练、筛选、导出及真机平衡控制；这些记录不能作为当前主线精确提交的完整验收。轻量测试、GPU/PhysX 运行、Windows ONNX 推理和硬件闭环分别验证不同层次。部署前按设备文档核对当前标定与会话配置，不把仓库中的历史设备档案当作实时硬件读数。

## 开发与排查

- 开发约束见 [AGENTS.md](AGENTS.md)。
- Python 格式和静态检查配置位于 `.pre-commit-config.yaml`。安装 `pre-commit` 后可运行 `pre-commit run --all-files`；这不代替仿真或硬件测试。
- Pylance 无法索引项目时，可在 VS Code 的 `python.analysis.extraPaths` 加入本项目 `source/cartpole`；Isaac Lab 和 Isaac Sim 的索引路径应对应实际安装位置。
- `ui_extension_example.py` 保留 Omniverse 扩展示例。独立训练不要求启用该界面扩展。
- [零动作吞吐对照与分段计时](scripts/diagnostics/README.md)不测量真实电机能力，也不衡量策略控制效果。
