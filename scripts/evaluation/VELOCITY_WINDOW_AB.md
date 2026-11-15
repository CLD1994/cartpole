# 冻结策略的速度观测 A/B（仅仿真）

复用 evaluate_policy.py，不训练、不改奖励/执行器/物理步长、不接设备。

- A: `--velocity-observation-window-ms 0`，保留原始策略观测。
- B: `--velocity-observation-window-ms 16`，仅将策略两路速度改为原始连续关节位置16ms首末差/16ms，保留既有速度噪声；位置输入不变。
- 物理仍1ms、策略仍2ms。9个控制采样端点的理想16ms窗近似固件17个1ms（或33个0.5ms）点的端点估速；不复制量化、采样抖动、失效或通信延迟。使用未包角关节位置，避免±π显示转换引入假跳变。
- 每次reset清历史，前16ms保留瞬时速度，不凭空制造历史或改变初始物理状态。完成槽位自动reset也清历史，不跨回合做差。
- 同模型、同seed、同num_envs、同其他参数运行两组。报告含初态及随机执行器参数，应先核对匹配；轨迹及终止分歧可能改变后续随机数消费，因此不声称后续逐点噪声严格配对。
- 指标仍读取无噪声物理状态。新增 action_delta_rms（相邻归一化动作差的RMS）及 pole_rms_velocity_rad_s；结合原车速RMS、角度RMS、峰值与失败判断。它们不直接定位14Hz频率，不能凭成功率不变就说抖动无变化。

PowerShell（`$checkpoint`设为实际已选的model_1050.pt完整路径；不使用ONNX路径）：

```powershell
$ab = ".\logs\velocity-window-ab\$(Get-Date -Format yyyyMMdd-HHmmss)"
python .\scripts\evaluation\evaluate_policy.py --checkpoint "$checkpoint" --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --velocity-observation-window-ms 0 --output-dir "$ab\instant"
if ($LASTEXITCODE -ne 0) { throw 'A evaluation failed' }
python .\scripts\evaluation\evaluate_policy.py --checkpoint "$checkpoint" --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --velocity-observation-window-ms 16 --output-dir "$ab\window16"
if ($LASTEXITCODE -ne 0) { throw 'B evaluation failed' }
Get-ChildItem $ab -Recurse -Filter report.json | Select-Object -ExpandProperty FullName
```

不需要Kit进行统计评估；可另用Kit观察，但不要把评估耗时当性能基准。64回合/组是初步敏感性比较，不是真机验收，也不是14Hz抖动已证实由估速窗导致。当前仅基本静态/局部数值核查，Isaac Lab现场待运行。
