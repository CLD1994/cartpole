# Cartpole 部署串口协议

USB 虚拟串口，**115200 8N1**，ASCII 行文本。两个方向：MCU → PC 是连续状态流，PC → MCU 是命令。

## MCU → PC：状态流（约 200 Hz）

```
S <cart_count> <pole_adc>\n
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `cart_count` | int32 | 小车编码器（装在**电机轴上**、减速箱之前）的正交计数原始值。正负号按小车 +x 方向标定。 |
| `pole_adc` | int (0-4095) | 摆杆电位器 12-bit ADC 原始值。 |

PC 每次读取最新的一行即可，不需要 ACK。

## PC → MCU：命令

```
P <pwm>\n
```

| 值 | 含义 |
|---|---|
| `pwm` 在 `[-1000, 1000]` | 有符号电机指令。符号 → DIR 引脚，绝对值 → PWM 占空比（千分比）。`P 0` = 停。 |

MCU 收到后立即更新 DIR + PWM 占空比，并保持到收到下一条 `P` 命令。

## 标定说明（在 PC 侧做，见 `bridge.py`）

- **小车位置 [m]** = `cart_count * meters_per_count`，
  `meters_per_count = pi * D / (G * PPR) = pi*0.0382/(20*52) = 1.154e-4 m/count`
  （2GT-60T 带轮 D=38.2mm，减速比 G=20，编码器 PPR=52）。**注意无 ×4 倍频**：TIM2 编码器接口每 PPR 线计 1 个，
  已用直尺实测验证（全程 ~3142 counts ≈ 0.363 m 可用行程，轨道 41 cm 减小车宽度）。
  bridge 启动时把计数清零，小车从 0 开始。
- **摆杆角度 [rad]** = `(pole_adc - POLE_ZERO_ADC) * RAD_PER_COUNT`，两个常量都来自电位器两点标定（`calibrate_pot.py`）。
- **速度**：由 PC 在 60 Hz 步长上做差分得到。
- **正负号必须和仿真一致**：仿真里 +action 驱动小车往 +x 走，摆杆竖直时关节角读数为 0。实机验证方法：发 `P 400`，确认小车移动方向与仿真 +action 一致；把摆杆顶端朝 +x 轻推，确认 `pole_angle` 变大。
