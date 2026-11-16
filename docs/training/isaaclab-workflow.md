# Isaac Lab 3.0 仿真工作流

## 从训练到部署

本流程只负责训练产物，不发送设备命令，也不更改固件参数。

- [训练指标](episode-metrics.md)：完整结束回合、失败与部分回合的统计口径。
- [动态候选与冻结策略筛选](../../scripts/evaluation/README.md)：默认保留三个候选、默认只评估已保存候选；有界并行后独立 holdout。
- [可选 400 Hz / Acc9 配方](400hz-acc9.md)：奖励、折扣、窗口观测、动作延迟的明确差异。

原 V2 任务保持回中奖励与原折扣。Window16 系列采用独立平衡配方。训练、评估与导出必须使用同一任务和配置；文件名不能代替配置核验。

训练指标只用于提名，冻结评估与 holdout 不能替代真机验证。主线流程只覆盖单动作，不包含双动作及其协议。单动作部署见 [设备运行器](../../deploy/v2/device/README.md)，支持 400/500 Hz 会话及固件 STEP_LOSS 事件；部署前仍需核对模型合同、设备配置与现场状态。不在训练工作区运行中切换代码。


## 适用环境

本说明以官方 Isaac Lab `v3.0.0`、提交 `e3f7d9ff1f5d7f8280de3ece9000b7117f66bcf4` 为准。源码环境中的 `isaaclab` 子包版本为 `28.0.0`，不应只看包版本数字来推断 Git 发布版本。

现场已确认 Python 3.12、Isaac Sim 6.2、RSL-RL 5.5.1，PyTorch CUDA 可用，本项目以 editable 方式安装在同一个 Isaac Lab Python 环境中。保持使用该解释器，不为命令行参数错误重建环境。

**不要在本仓库根目录直接执行 `uv sync` 或依赖自动同步的 `uv run`。** 根 `pyproject.toml` 仍是上游开发环境布局，含机器专用的源码路径及不同的 PyTorch 固定版本，并非可直接复用的独立项目环境。官方文档中的 `uv run` 示例以正确配置的 Isaac Lab 或外部项目环境为前提。

## 入口及任务注册

本项目使用 RSL-RL。单动作 V2 提供原始任务 `Hexchip-Cartpole-Real-V2-v0`，以及 Window16、Window16-Tau0、Window16-Tau0-Acc9-Hz400 三种独立配方。任务名和差异见 [400 Hz 配方](400hz-acc9.md)。训练、筛选、回放导出必须显式选择同一任务；原始 V2 不是 400 Hz 配方的别名。

- `scripts/rsl_rl/train.py`：初始化 Warp、注册本项目任务，调用项目 `train_rsl_rl.run`，接入完整回合日志和可选动态候选保存。
- `scripts/rsl_rl/play.py`：注册本项目任务，调用公开 `run_play_cli`。
- 项目训练后端复用 Isaac Lab 3.0 的公开配置、启动、环境、runner 和清理函数；不修改 Isaac Lab 或 RSL-RL 安装文件。回放仍调用官方 `run_play_cli`。

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

`--headless` 不是此版本公开 CLI 参数。本项目不增加私有兼容别名，以免两套参数语义长期并存。项目候选参数由训练后端解析；其余参数沿用 Isaac Lab CLI 及配置覆盖机制，未知参数错误应被明确暴露。

“无窗口”不等于“不启动 Kit”或“不渲染”。本项目默认物理后端为 Isaac Sim PhysX，仍需要相应运行时；相机或视频可以在无交互窗口时渲染。

保持本项目默认 PhysX 基线。V2 速度跟踪动作要求每个物理子步执行动作更新，不能仅因官方示例默认使用 Newton 就替换本任务后端。项目中的 `physics=physx` 直接选择 `PhysxCfg`，不代表其他任务里的自动 PhysX 后端选择器。

## 终端输出与日志

`--viz none` 不关闭终端启动界面。官方在交互终端使用 Rich 的 alternate screen 显示启动进度，这不是持久日志；它可能影响启动期间的滚动查看。需要普通文本时增加 `--info`，或将输出重定向到文件。两种方式都会禁用该启动界面，不需要修改终端设置。

需要保留完整启动输出时，可为下方有限训练命令增加 `--info *> .\train-smoke.log`。命令结束后立即保存 `$trainExit = $LASTEXITCODE`，然后读取日志；不要先运行其他原生命令覆盖退出码。重定向时控制台没有实时输出是预期行为。

`SimulationContext cleared` 只表示执行过清理，退出码 0 也不能单独证明训练完成。应结合本次迭代记录、异常信息和时间对应的新模型文件判断，不能把旧检查点当作本次成功证据。若加了 `--info` 后仍无法滚动，应保留具体终端和日志信息，不能直接认定仍由启动界面引起。

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

回放通过后才能安排更长的训练、比较收益曲线和策略稳定性。仿真策略可运行仍不代表能部署到实物；设备与建模边界见 [训练准备](sim2real-model.md)。

## 已核查与待验证

已按精确版本源码核查：公开训练/回放入口、CLI 参数、配置解析与启动顺序、Kamino 嵌套配置、质量读取接口、动作与事件方法、代理配置字段。轻量测试覆盖命令转发及设备/训练参数，不调用真实仿真器。

### 验证范围

主线包含项目训练后端、完整回合指标、动态候选、冻结评估，以及 400 Hz 单动作配方。2026-11-15 的现场日志记录了相关实验分支完成训练、检查点筛选和 JIT/ONNX 导出；这些记录不等于主线精确提交及其全部任务已经重新通过 GPU/PhysX 验收。

每次验证应保存代码提交、任务、实际配置、模型来源和日志。导出成功不代替可视回放，仿真回放不代替当前固件及硬件的部署验证。本文档更新没有新增仿真运行结果。其他诊断脚本的适用范围见各自说明。

## 官方依据

- [训练、日志和模型回放](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/concepts/reinforcement_learning.rst)
- [Windows 安装与 uv 环境](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/isaaclab/installation.rst)
- [物理后端、渲染与预设](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/docs/source/concepts/backends_and_presets.rst)
- [公开启动参数和无窗口选择](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_physx/isaaclab_physx/app/kit_launcher.py)
- [官方 RSL-RL 训练实现](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/train_rsl_rl.py)
- [官方 RSL-RL 回放实现](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/source/isaaclab_rl/isaaclab_rl/entrypoints/backends/play_rsl_rl.py)
- [外部项目任务入口模板](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0/tools/template/templates/external/pyproject.toml.jinja)
