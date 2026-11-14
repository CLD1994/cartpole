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

默认预热500步，测量5000步，单环境。物理后端固定为 Isaac Sim PhysX；两组只改变是否启用界面。实际物理步长、控制周期、渲染间隔会随结果输出。默认保留任务配置。

默认不 sleep，测量可达到的回放速度。追加 `--real-time` 才模拟官方 play 的补 sleep 节拍；慢于实时则不补 sleep，也不跳步追赶。两种模式必须分别比较。

固定种子用于提高可重复性，不承诺GPU逐位确定性。和官方play一致，加载当前任务配置而不是模型目录里的 env.yaml；play_mode关闭观测噪声，但保留reset随机化。

只有 KIT 组显著更慢时，才支持显示路径带来额外开销的判断；这仍不足以定位具体渲染或UI模块。两组都慢时应继续检查仿真、Python和GPU同步。速度比不能证明模型几何或动力学正确。

## 验证边界

计时单元通过注入时钟、步进和同步函数验证，无需Isaac Lab。GPU入口依照Isaac Lab 3.0公开接口编写，仍需在真实Isaac运行环境验证后使用其测量结果；单元测试不等于GPU验证。

```text
python -m unittest discover -s scripts/diagnostics -p "test_*.py"
```

## 调用分段

追加 `--breakdown` 可观察耗时集中在哪个主机调用：

- `policy`：策略推理的主机调用。
- `environment`：整个环境步进，包括物理、观测、奖励、回合重置及可能的渲染。
- `policy_recurrent_reset`：策略循环状态重置，不是环境回合重置。

每项输出次数、累计、平均、p50、p95、最大耗时。统计不含预热，分位数为最近秩定义。排序和JSON输出在计时外执行。记录每次主机调用增加少量时钟读取和列表写入；需要与同条件不带 `--breakdown` 的RTF对照，不能把诊断开销当成模型变化。

GPU执行是异步的：某调用可能只提交工作，后续调用才等待前面的GPU结果。因此这些值不能称为GPU核耗时，也不能把环境调用的等待全部归咎物理。工具不在各阶段强制同步。`completion_sync_s`仅表示循环结束时仍需等待的工作，并不是全部GPU时间。`step_loop_wall_s`加`completion_sync_s`等于总墙钟时间。

GUI/无GUI对照须使用相同模型、种子、步数、预热和节拍模式。可先用 `--steps 1000 --warmup 500 --breakdown` 缩短诊断轮次，再依据数据决定是否需要更深入的独立短时性能跟踪；不要修改物理步长、控制频率或模型来掩盖吞吐不足。

## 显示间隔对照

`--render-interval` 指定每隔多少个物理步更新显示，必须为正整数；省略时保留任务默认值。
例如物理步长为 0.001 秒时，`--render-interval 20` 表示每 0.02 秒模拟时间更新一次显示。
这不改变物理步长、控制周期或模型，也不保证现实时间内恒定 50 帧。

```powershell
python .\scripts\diagnostics\playback_rtf.py --checkpoint <模型完整路径> --device cpu --viz kit --info --render-interval 20
```

与基线保持相同设备、模型、种子、步数和预热，只改变显示间隔。核对输出中的 `render_interval`，同时观察界面响应与 RTF。

## 预热后的短窗口函数采样

`--profile <输出文件>` 使用 Python 标准库 cProfile，仅采样预热后的 `--steps` 个控制步。
默认不启用，无需安装额外依赖。启动、模型加载、预热、最终设备同步和输出文件写入不在采样窗口内。
采样包含控制循环自身与调用的主机函数。步进失败时停止采样并传播异常，不生成成功报告。
输出父目录须已存在；指定的同名文件会被覆盖。

```powershell
python .\scripts\diagnostics\playback_rtf.py --checkpoint <模型完整路径> --device cpu --viz newton_gl --render-interval 20 --steps 500 --warmup 500 --seed 42 --info --profile .\playback-cpu-gl.prof
python -c "import pstats; pstats.Stats('playback-cpu-gl.prof').strip_dirs().sort_stats('cumulative').print_stats(60)" *> .\playback-cpu-gl-summary.txt
```

cProfile 会增加主机调用开销，报告中的 RTF 不能作为正常运行性能基线。
累计时间包含子调用，不能把各行累计时间直接相加。先用累计时间定位显示、状态传输或窗口呈现路径，再结合自身时间判断。
它不能区分 GPU 内核执行与驱动等待，也不覆盖其他线程中的完整工作。结果仅用于定位下一次针对性实验，不能据此改变训练、物理步长或控制频率。
