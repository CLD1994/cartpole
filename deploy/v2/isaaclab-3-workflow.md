# Isaac Lab 3.0 仿真工作流

## 适用环境

本说明以官方 Isaac Lab `v3.0.0`、提交 `e3f7d9ff1f5d7f8280de3ece9000b7117f66bcf4` 为准。源码环境中的 `isaaclab` 子包版本为 `28.0.0`，不应只看包版本数字来推断 Git 发布版本。

现场已确认 Python 3.12、Isaac Sim 6.2、RSL-RL 5.5.1，PyTorch CUDA 可用，本项目以 editable 方式安装在同一个 Isaac Lab Python 环境中。保持使用该解释器，不为命令行参数错误重建环境。

**不要在本仓库根目录直接执行 `uv sync` 或依赖自动同步的 `uv run`。** 根 `pyproject.toml` 仍是上游开发环境布局，含机器专用的源码路径及不同的 PyTorch 固定版本，并非可直接复用的独立项目环境。官方文档中的 `uv run` 示例以正确配置的 Isaac Lab 或外部项目环境为前提。

## 入口及任务注册

本项目的任务名为 `Hexchip-Cartpole-Real-V2-v0`，使用 RSL-RL。

- `scripts/rsl_rl/train.py`：注册本项目任务，调用公开 `run_train_cli`。
- `scripts/rsl_rl/play.py`：注册本项目任务，调用公开 `run_play_cli`。
- 参数解析、仿真启动、环境与 runner 创建、保存和关闭由 Isaac Lab 负责。

本项目的安装配置尚未声明 `isaaclab.tasks` 包入口，因而不要直接假设裸 `isaaclab train` 已能发现本项目任务。当前脚本通过显式导入 `cartpole.tasks` 完成注册。

官方流程先解析环境和代理配置，再按配置启动所需运行时，最后创建环境。配置解析成功并不意味着仿真已经启动；不能为了消除一条导入警告而自行提前创建第二个 `SimulationApp`。

## 参数规则

| 目标 | 参数或规则 |
| --- | --- |
| 不显示交互窗口 | 默认不指定 visualizer，或显式 `--viz none` |
| 查看 Isaac Sim 视口 | `--viz kit` |
| 训练迭代数 | `--max_iterations 10` |
| 并行环境数 | `--num_envs 64` |
| 加载模型 | `--checkpoint <模型路径>` |
| 仿真配置覆盖 | Hydra token，例如 `env.seed=42`，不加 `--` |
| 选择已声明的物理预设 | 例如 `physics=physx`，不加 `--` |

`--headless` 不是此版本公开 CLI 参数。本项目不增加私有兼容别名，以免两套参数语义长期并存。未知参数原样交由官方解析，错误应被明确暴露。

“无窗口”不等于“不启动 Kit”或“不渲染”。本项目默认物理后端为 Isaac Sim PhysX，仍需要相应运行时；相机或视频可以在无交互窗口时渲染。

保持本项目默认 PhysX 基线。V2 速度跟踪动作要求每个物理子步执行动作更新，不能仅因官方示例默认使用 Newton 就替换本任务后端。项目中的 `physics=physx` 直接选择 `PhysxCfg`，不代表其他任务里的自动 PhysX 后端选择器。

## 分阶段验证

以下是准备好的工作流，不代表这些阶段已经通过。先完成前一阶段，再进入后一阶段；不连接串口，不控制真机。

### 1. 命令和配置

在已安装本项目的 Isaac Lab 环境中，检查任务专属帮助：

```powershell
python scripts/rsl_rl/train.py --task Hexchip-Cartpole-Real-V2-v0 --help
```

此阶段确认任务可发现、配置可加载、参数名称与当前版本一致，不启动强化学习训练。

### 2. 有限仿真训练

从本仓库目录运行：

```powershell
python scripts/rsl_rl/train.py --task Hexchip-Cartpole-Real-V2-v0 --num_envs 64 --max_iterations 10 --viz none
```

成功标准：环境创建和首次 reset 成功、PPO 完成 10 次迭代、生成模型检查点，且没有未处理异常。仅打印环境和代理配置路径不算通过。

日志根目录是 `logs/rsl_rl/cartpole_real_v2/`，每次运行有独立子目录。保存该次运行的模型与环境/代理配置，后续回放使用明确的模型路径，避免误选其他实验。

### 3. 回放模型

```powershell
python scripts/rsl_rl/play.py --task Hexchip-Cartpole-Real-V2-v0 --checkpoint <本次模型路径> --num_envs 1 --viz kit
```

回放会加载模型并导出 JIT/ONNX 文件，再执行策略；模型导出也可能暴露依赖问题。普通回放没有有限步数截止，检查完成后用 Ctrl+C 结束，不把它当作自动结束的冒烟测试。需要录视频时再单独配置录制长度和相关依赖。

回放通过后才能安排更长的训练、比较收益曲线和策略稳定性。仿真策略可运行仍不代表能部署到实物；设备与建模边界见 [训练准备](training-readiness.md)。

## 已核查与待验证

已按精确版本源码核查：公开训练/回放入口、CLI 参数、配置解析与启动顺序、Kamino 嵌套配置、质量读取接口、动作与事件方法、代理配置字段。轻量测试覆盖命令转发及设备/训练参数，不调用真实仿真器。

仍待现场验证：GPU 环境创建、资产加载、首次 reset、动作/奖励/PPO 更新、检查点保存与回放、运行吞吐及训练效果。其他诊断脚本尚未纳入本次完整兼容核查。

## 官方依据

- [训练、日志和模型回放](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/concepts/reinforcement_learning.rst)
- [Windows 安装与 uv 环境](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/isaaclab/installation.rst)
- [物理后端、渲染与预设](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/concepts/backends_and_presets.rst)
- [公开启动参数和无窗口选择](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_physx/isaaclab_physx/app/kit_launcher.py)
- [官方 RSL-RL 训练实现](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/train_rsl_rl.py)
- [官方 RSL-RL 回放实现](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/play_rsl_rl.py)
- [外部项目任务入口模板](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/tools/template/templates/external/pyproject.toml.jinja)
