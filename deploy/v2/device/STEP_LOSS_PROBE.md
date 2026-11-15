# 单串口 STEP_LOSS 事件低速恢复探针

仅用于有人在场的现场诊断，显式 `--step-loss-probe --continuous --start-confirmed` 才启用；默认运行器行为不变。只支持 400 Hz 单动作模型。固件须使用 `device-nerve-esp32` 的 `fix/changqing-wdt-steptrace` 分支 支持 `RUN?` 和每次 ENABLE 重新布防的版本，显式开启 STEP_LOSS 仅记录模式，并支持一次异步通知：

```
EVENT STEP_LOSS <event_id> <run_id> <device_us>
```

`device_us` 是固件锁存事件的时间，不是上位机接收时间。通知不是 `MODE FAULT`，它不会禁用电机。无 ACK，串口断开/丢失时不保证送达或固定延迟。收到明确的新事件才切换，不以位置不变猜测失步。启动先用 `RUN?` → `RUN <run_id>` 确认当前运行编号，能力缺失即拒绝运行。启动阶段收到的事件只作既有记录；ACTIVE 后只有同 run 的新事件触发。未收到事件时仍由模型控制，但最多等待 120 秒，到期正常停车并报告 `event_not_received`，不会声称执行了低速探针。接受事件后完整执行 5+2 秒，不被等待截止截短。120 秒不是倒杆保护，仍需现场 Ctrl+C 停止。

同一进程、同一串口、同一会话：模型 → 新事件 → 固定正方向 +10 mm/s、最多约 5 秒的非零窗口 → VEL 0 并继续观察 2 秒 → 原有停机确认及 DISABLE。不会自动 RESET 或重新 ENABLE。窗口是主机计划，不是物理停车上界；线程迟延、写入/传输失败仍可能改变实际时间，已有超时和固件看门狗仍生效。理想恒速位移约 50 mm，不是位移上界。请人工核实正方向有足够净空，并准备独立断电。

此显式调试选项使用命令间隔硬限 15 ms、观测年龄 12 ms，软告警 4 ms；不修改固件看门狗或其它配置。报告记录实际门限、事件及上位机接收时间、探针阶段和 STATE。比较固件冻结记录时必须匹配 event_id/run_id；不能拿上次启动的事件解释本次恢复。每次成功 ENABLE 重新布防，只接受启动 RUN? 读回的运行编号；事件仍可能丢失，事后核对冻结事件编号。

运行示例（已有 `$python/$onnx/$contract/$sessionProfile`，另按既有流程确认 READY）：

```powershell
$report = Join-Path (Split-Path $onnx -Parent) ('event-probe-' + [guid]::NewGuid().ToString('N') + '.json')
& $python .\deploy\v2\device\run_policy.py --port COM11 `
  --onnx $onnx --contract $contract --profile $sessionProfile `
  --output $report --continuous --step-loss-probe --start-confirmed
```

命令结束后检查报告中停机是否确认。停稳 DISABLED 后，再用 STEPLOSS?/STEPTRACE? 读本次固件冻结证据；不要在持续控制期间争用 COM11 或拉取大段轨迹。
