# Cartpole 固件改造实施计划（PD-STEP → protocol.md 线协议）

> **执行者注意：** 必须配合 superpowers:subagent-driven-development（推荐）或 superpowers:executing-plans 技能逐任务执行本计划。步骤用复选框（`- [ ]`）跟踪。
>
> 本计划的行为契约唯一权威是 [deploy/v2/protocol.md](protocol.md)；执行者动手前先通读它。本文区分三类内容：**契约**（协议规定，不可偏离）、**项目规则**（本仓库既定约束）、**预估**（实现方式建议，允许执行者按实际代码结构调整）。

**目标：** 把 PD-STEP（ESP32-S3 单轴 G-code 控制器，固件仓库 `tmp/PD-STEP`）重构为 cartpole 固件——线协议完全按 protocol.md 执行，新增 VEL 速度模式、堵转回零、四态模式机、200 Hz STATE 遥测与 MT6835 摆角编码器。

**架构：** 保留 bsp/tmc2209/as5600/settings/step_engine 的告警链与位置计数基础设施，删除 G-code 人格（gcode_parser/planner/sysctl/monitor）；step_engine 的段队列替换为带符号目标速度的 ISR 递推；新增 mt6835、cartproto、cartfsm、telemetry 四个组件。数据面与控制面共用 UART1（460800）但分属 telemetry 与 cartproto，发送串口只有一个属主（cartproto）。

**技术栈：** ESP-IDF v6.1（本机 `C:\esp\v6.1\esp-idf`，PowerShell `export.ps1` 后用 `idf.py`）、FreeRTOS、GPTimer/SPI/I2C 外设驱动、C11。

**规格：** `deploy/v2/protocol.md`（线协议权威）、本计划「既有事实」节（固件接口事实）。

## 全局约束

- 线协议行为以 protocol.md 为唯一权威；任何与固件现状冲突处，固件向协议靠。固件回复行结尾 `\n`（edge 容忍可选 `\r`）。
- 波特率 460800 8N1；UART1 = GPIO41(TX)/42(RX)。
- 摆角编码器 MT6835，SPI：MISO=IO46、SCK=IO39、MOSI=IO40、CS=IO7。
- 细分 8（40 steps/mm @40 mm/圈）；VEL_MAX 1000 mm/s → 40000 steps/s。
- 固件按 protocol.md §参数注册表 默认值上电；参数权威在端侧（会话推送）。
- 上电默认 DISABLED、电机不使能、STATE 流在所有模式持续发送。
- AGENTS.md 测试规范适用：先写可证伪契约的最窄测试再实现；测试失败先定位根因；不删测试不放宽配置让门禁变绿。
- 沿用固件仓库 T 系列方法论：每任务 Kconfig 门控自测（上板执行，最终日志行 `SELF-TEST DONE: <n>/<total> PASS`），验收记录写入固件仓库 `docs/reviews/T<n>-*.md`。
- 提交信息与注释用中文完整句子；最终产物不保留被否决方案的痕迹（G-code 人格的删除不留过渡性兼容层）。

## 既有事实（复用基线，已核对源码）

| 事实 | 位置 |
|---|---|
| step_engine ISR：GPTimer 1 MHz、one-shot alarm、`arm_alarm(base, floor, dt)`、`isr_set_alarm` 超过 `MAX_ALARM_US` 走链式等待、DIR 换向加 2 µs、Q16.16 速度、u64 中间量、`portMUX_TYPE` 自旋锁、位置计数 `s_position`（含 $10 反转）、ISR 钉在 CPU0 | `components/step_engine/step_engine.c` |
| as5600：1 kHz 采样任务（CPU1、prio 3、vTaskDelayUntil）、多圈 tracker（±2048/-2047 非对称阈值）、16 样本速度窗口、`as5600_is_ok()`（连续 ≥10 次 I2C 失败置 false）、`as5600_position_counts()/velocity_mm_s()/set_origin(mm)`、9.765625 µm/count | `components/as5600/` |
| settings：NVS 表 `$0`–`$13`（见下文任务 2 精简）、订阅分发 `settings_subscribe(cb)` | `components/settings/` |
| tmc2209：CRC8 寄存器读写、`set_current/set_microstep/enable`、`read_status`；`REG_SGTHRS=0x40`、`REG_SG_RESULT=0x41` 已定义未使用 | `components/tmc2209/` |
| bsp：`bsp_early_init()`（最早拉高 EN）、`bsp_init()`（UART1 驱动/GPIO/I2C+扫描）、`bsp_get_i2c_bus()`、`bsp_tmc_en_set()`；DIAG=GPIO16 已配输入 | `components/bsp/` |
| 协议任务骨架：UART1 事件队列驱动、行组装、TX 互斥 `protocol_send_raw` | `components/protocol/protocol.c`（任务 1 更名改造为 cartproto） |
| 任务看门狗：`CONFIG_ESP_TASK_WDT_PANIC=y`，各任务订阅喂狗 | `sdkconfig.defaults` |

## 未决事项（执行中须停下确认，不得静默选择）

1. **MT6835 SPI 读帧格式**：角度寄存器地址、命令字节、角度位域须从 MT6835 数据手册抄录并写进测试向量（任务 4 步骤 1）。不得凭记忆编造帧格式。
2. **StallGuard 低速可靠性**：HOME_VEL 50 mm/s ≈ 1.25 rps 可能低于 StallGuard 有效区。联调时若不触发，按 protocol.md 未决事项路径调整（提速/换 C−E 判据）。
3. `SGTHRS` 默认 40 与 `STEP_LOSS_UM` 默认 3000 是待实机标定的初值。
4. 460800 两端联调（protocol.md 未决事项沿用）。

台位标定量（`POLE_ZERO_URAD`/`POLE_INV`/`X_INV`/`SGTHRS`/`STEP_LOSS_UM`）已批准扩入 protocol.md 参数注册表（2026-09-23 修订），不是未决事项；标定初值在实机验收时确定并写回端侧 `config.h`。

## 目标组件结构（预估，允许执行者微调归属，接口签名不得擅改）

```
components/
  bsp/          保留 + SPI 主机初始化（SPI3_HOST：MOSI=40 MISO=46 SCK=39）+ CS(IO7)
  tmc2209/      保留 + tmc2209_set_stall_threshold()
  as5600/       原样
  mt6835/       新：21-bit 角读取 → ±π 环绕 µrad + 速度估计 + is_ok
  step_engine/  段队列删除；速度模式（带符号目标 + 递推 + 看门狗）
  cartproto/    新（由 protocol/ 改造）：行组装/解析/分发/MODE/OK/ERR + CONFIG 注册表
  cartfsm/      新：四态模式机 + 1 kHz 监督任务 + 回零 + 使能控制
  telemetry/    新：200 Hz STATE 组帧（经 cartproto 发送）
  settings/     精简为机械参数 + NVS namespace 更名
main/app_main.c 重写
删除：gcode_parser/ planner/ sysctl/ monitor/ protocol/(旧身) main/app_*_test.c
```

任务/CPU 布局（预估）：CPU0 = cartproto 通讯任务（prio 5）+ STEP ISR；CPU1 = as5600 1 kHz（prio 3）、mt6835 1 kHz（prio 3）、cartfsm 监督 1 kHz（prio 4）、telemetry 200 Hz（prio 2）。全部订阅任务看门狗。

---

### 任务 1：构建骨架切换（删除 G-code 人格，v6.1 下可编译烧录）

**文件：**
- 删除：`components/gcode_parser/`、`components/planner/`、`components/sysctl/`、`components/monitor/`、`main/app_joint_test.c`、`main/app_m3_test.c`
- 更名：`components/protocol/` → `components/cartproto/`（`protocol.c/h` → `cartproto.c/h`，`git mv` 保留历史）
- 修改：`CMakeLists.txt`（顶层与 main/）、`main/app_main.c`（重写为最小骨架）、`main/Kconfig`（删测试开关）、`sdkconfig.release`（删除，后续重新生成）
- 修改：`components/cartproto/cartproto.c` 清空 GRBL 实时字符（`?`/`!`）与 ok/error 应答，仅保留 UART1 事件驱动收发骨架与 TX 互斥

**接口（产出，后续任务依赖）：**
```c
/* cartproto.h —— 任务 1 只交付收发骨架；解析与分发在任务 6 填充 */
esp_err_t cartproto_start(void);            /* 创建通讯任务（CPU0, prio 5, 4KB），460800 8N1 */
void cartproto_send_raw(const char *data, int len);  /* TX 互斥保护的原始发送（不含结尾） */
/* 行回调注册：每凑成完整 '\n' 行回调一次，行内不含 '\r'/'\n' */
typedef void (*cartproto_line_cb_t)(const char *line);
void cartproto_register_line_handler(cartproto_line_cb_t cb);
```

- [ ] **步骤 1：删除退役组件与测试载体**。`git rm -r` 上列目录/文件；顶层 `CMakeLists.txt` 的 `EXTRA_COMPONENT_DIRS` 无需变（components/ 目录自动扫描），但 main/CMakeLists.txt 的源列表里删掉 `app_joint_test.c`/`app_m3_test.c`。
- [ ] **步骤 2：`git mv components/protocol components/cartproto` 并改文件名**，全局把 `protocol.h` include、`protocol_start/send_raw/register_*` 标识符改为 `cartproto_*`（仅机械更名，逻辑不动）。删掉 `?`/`!` 实时字符处理与 `protocol_send_ok/send_error`；波特率来源改为 `CONFIG_CARPROTO_UART_BAUD`（新 Kconfig，默认 460800；BSP Kconfig 里旧波特率项删除）。
- [ ] **步骤 3：重写 `main/app_main.c` 最小骨架**（删除日志镜像块——自测输出走 UART0 控制台即可）：

```c
#include "esp_log.h"
#include "bsp.h"
#include "settings.h"
#include "tmc2209.h"
#include "step_engine.h"
#include "as5600.h"
#include "cartproto.h"

void app_main(void)
{
    bsp_early_init();                    /* EN 高 = 电机不使能 */
    ESP_ERROR_CHECK(settings_init());
    esp_err_t bsp_err = bsp_init();
    if (bsp_err == ESP_OK) {
        ESP_ERROR_CHECK(as5600_init());
        tmc2209_config_t tmc = {
            .uart_num = UART_NUM_2,
            .tx_pin = CONFIG_BSP_TMC_UART_TX_GPIO,
            .rx_pin = CONFIG_BSP_TMC_UART_RX_GPIO,
            .baud = 115200,
            .slave_addr = 0,
            .r_sense = CONFIG_TMC2209_RSENSE,
        };
        ESP_ERROR_CHECK(tmc2209_init(&tmc));
        ESP_ERROR_CHECK(cartproto_start());
        ESP_ERROR_CHECK(step_engine_init(CONFIG_BSP_STEP_GPIO, CONFIG_BSP_DIR_GPIO));
    }
    ESP_LOGI("app_main", "cartpole firmware up (protocol skeleton)");
}
```

- [ ] **步骤 4：v6.1 构建验证**。PowerShell：`C:\esp\v6.1\esp-idf\export.ps1` 后，删除旧 `sdkconfig`，`idf.py -B build build`。若出现 v6.0.2→v6.1 的驱动 API 弃用告警，逐条修复（uart/gptimer/i2c_master 在 6.x 内稳定，预期无碍；若 SPI 需要新头文件在任务 4 处理）。**预期：零错误。**
- [ ] **步骤 5：上板烧录冒烟**（板在位时）：`idf.py -B build flash monitor`，确认启动横幅、`as5600 sampling task up`、无 panic、UART1 无主动输出（骨架不发数据面）。**预期：日志正常，EN 保持高。**
- [ ] **步骤 6：提交** `git commit -m "重构：删除 G-code 人格，协议组件更名 cartproto，最小骨架在 v6.1 下构建通过"`

### 任务 2：settings 精简与默认值更新

**文件：**
- 修改：`components/settings/settings.c`（参数表）、`include/settings.h`、`Kconfig`（自测开关保留）、`settings_self_test.c`
- 修改：`sdkconfig.defaults`（无 settings 相关项则不动）

**接口（产出）：**
```c
/* 机械/驱动层参数，NVS namespace "cart"（旧 "pdstep" 弃用 → 升级即冷启动默认值）。
 * 注意：方向反转不在此表——它由 CONFIG 扩项 X_INV 承担（见未决事项 1），
 * 避免 NVS 与 CONFIG 两处权威。 */
enum {
    SET_MICROSTEP = 0,   /* 8（默认由 16 改），2^n, 1–256, 整数 */
    SET_FULL_STEP = 1,   /* 200, ∈{200,400} */
    SET_MM_PER_REV = 2,  /* 40.0, (0,500] */
    SET_RUN_MA = 3,      /* 800, (0,2000], 整数 */
    SET_HOLD_MA = 4,     /* 400, [0,$3], 整数 */
    SETTINGS_COUNT
};
/* 访问器/订阅器签名沿用现有 settings_get/set/is_valid/name/count/subscribe */
```

- [ ] **步骤 1：先改自测**。`settings_self_test.c` 中删除 $5–$13 相关用例，新增：「默认 microstep == 8」「SET_HOLD_MA > SET_RUN_MA 拒绝」。运行（上板自测门控构建）确认新用例失败。
- [ ] **步骤 2：实现**。按上表重写 `s_meta[]` 与 `settings_is_valid()`；NVS namespace 常量改 `"cart"`；`app_main` 若引用已删参数（default_feed 等）一并清理。微步默认改 8 同步改 `tmc2209` 初始化取值路径（它已订阅 $0，无需额外改）。
- [ ] **步骤 3：构建 + 上板自测**，`SELF-TEST DONE` 全 PASS（旧 NVS 内 16 细分因 namespace 更新自动回落默认 8——正是目的）。
- [ ] **步骤 4：提交** `git commit -m "settings 精简为机械参数层：细分默认 8，NVS namespace 更名 cart"`

### 任务 3：step_engine 速度模式（本计划核心）

**文件：**
- 修改：`components/step_engine/step_engine.c`（删段队列，加速度模式）、`include/step_engine.h`、`step_engine_self_test.c`、`Kconfig`

**接口（产出，cartfsm 依赖全部签名）：**
```c
/* 速度模式：目标带符号（Q16.16 steps/s），|v| 上限 65535（细分 8 下 40000 steps/s 余量足够） */
void step_engine_vel_set_accel(uint32_t accel_steps_s2);   /* ACC_MAX，任务上下文 */
void step_engine_vel_set_decel(uint32_t decel_steps_s2);   /* WDT_DEC，任务上下文 */
void step_engine_vel_set_wdt_us(uint32_t wdt_us);          /* WDT_MS，0 = 关闭看门狗 */
void step_engine_vel_target(int32_t v_q16);                /* 设定目标并按需启动定时器；喂狗 */
void step_engine_vel_halt(void);                           /* 目标清 0（斜坡停，非急停） */
int32_t step_engine_vel_now_q16(void);                     /* 当前速度快照 */
/* 保留：step_engine_init / step_engine_stop（急停，清一切）/ position_steps /
 *       set_dir_invert / 自测采样钩子。删除：step_seg_t / enqueue / busy。 */
```

**ISR 递推（契约级设计，实现不得改变语义）：** 每发出一步后更新速度再排下一拍。当前方向 `d`、当前速率 `|v|`（uint32 Q16.16）、目标拆成 `(dir_t, |v_t|)`：

- 减速段：`dv = a / v`（a 为 steps/s² 整数，u64 中间量 `(uint64_t)a << 16 / v_q16`），`v' = (dv >= v) ? 0 : v - dv`；
- 加速段：`v' = v + dv`，钳位到目标 `|v_t|`（不过冲）；`v == 0` 起步用预算 `dt0_us = sqrt(2e6/a)`（set_accel 时任务上下文算好）；
- 下一拍间隔 `dt_us = (uint64_t)1000000 * 65536 / v'`（v'=0 时停表：`s_stopped = true`，模式机不动）；
- 换向：目标方向与当前方向相反时，先把 `|v|` 斜坡到 0，到位且目标仍非零，再换 DIR（沿用 2 µs setup）向新方向起步——「换向经零速」；
- 看门狗：ISR 内比较 `gptimer` 原始计数（µs）与 `last_kick_us + wdt_us`；超时则有效目标按 0、加速度用 `decel` 处理（**保持 READY 语义由 cartfsm 保证，引擎只管停**）。`vel_target()` 每次调用刷新 `last_kick_us`。

- [ ] **步骤 1：抽纯函数 + 先写失败测试**。新增 `vel_ramp` 纯函数（无硬件，输入输出全值）：

```c
/* 每步速度递推。返回 true 表示继续运动，false 表示已到零速停表。
 * out_dt_us 为下一拍间隔；out_dir_change 表示本次到零且目标反向需换 DIR。 */
bool vel_ramp_next(uint32_t v_q16, bool dir, uint32_t target_q16, bool target_dir,
                   uint32_t accel, uint32_t decel, bool wdt_expired,
                   uint32_t *out_v_q16, uint32_t *out_dt_us, bool *out_dir_change);
```

自测向量（手算精确值，逐条断言）：①加速段：v_q16=6553600（100 steps/s）、a=2000 steps/s² → dv=a/v=20 steps/s，v'=120 steps/s（v'_q16=7864320），下一拍 dt'=1e6/120≈8333 µs；②减速到零：v=5 steps/s，dv=20 → v'=0，返回 false；③加速钳位：target=110，v=100，dv=20 → v'=110 恰在目标；④看门狗超时用 decel 而非 accel；⑤零速起步由调用方用 dt0（函数对 v=0 直接返回 dt0 语义由 `vel_ramp_start_dt(accel)` 单独给出）。运行自测确认失败（函数不存在）。
- [ ] **步骤 2：实现 `vel_ramp` 与新 ISR 分支**。删除段队列/`try_pop_segment`/相位机；保留 `arm_alarm/isr_set_alarm/链式等待/DIR setup/位置计数/自旋锁`。`vel_target` 用 `portENTER_CRITICAL(&s_spinlock)` 处理「定时器已停且目标非零」的启动竞态（种子 `s_cb_alarm_base` 沿用 enqueue 启动路径的冻结计数法）。
- [ ] **步骤 3：自测扩展到 ISR 层**（沿用 `step_selftest_start_sampling` 间隔采样钩子）：①目标 200 steps/s、a=2000：采样间隔单调上升至 5000 µs 稳定；②看门狗：设 wdt=100000 µs 后不再发 target，速度应在 ~200 ms 内归零且 `vel_now`==0；③换向：+200 → −200 过零处采样间隔出现一次 dt0 量级的长拍且无丢步（总步数守恒：位置变化 = Σ±1）。
- [ ] **步骤 4：构建 + 上板自测**，`SELF-TEST DONE` 全 PASS。
- [ ] **步骤 5：提交** `git commit -m "step_engine 改造为速度模式：带符号目标递推 + 看门狗减速，删除段队列"`

### 任务 4：mt6835 摆角编码器组件

**文件：**
- 修改：`components/bsp/bsp.c/.h`（SPI 主机初始化）、`Kconfig`
- 新建：`components/mt6835/`（`mt6835.c/h`、`mt6835_self_test.c`、`CMakeLists.txt`、`Kconfig`）

**接口（产出，telemetry 与 cartfsm 依赖）：**
```c
esp_err_t mt6835_init(void);          /* SPI 设备 + 1 kHz 采样任务（CPU1, prio 3），幂等 */
int32_t  mt6835_theta_urad(void);     /* ±π 环绕，直立标定零位由 CONFIG POLE_ZERO/POLE_INV（任务 7）应用前的原始角 */
int32_t  mt6835_vth_mrad_s(void);     /* 16 样本窗口速度估计 */
bool     mt6835_is_ok(void);          /* 连续 ≥10 次 SPI 失败置 false */
/* 纯函数（自测目标）：21-bit 原始码 → ±π µrad 环绕 */
int32_t  mt6835_raw_to_urad(uint32_t raw21, int32_t zero_urad, bool invert);
```

- [ ] **步骤 1：抄数据手册（未决事项 2 的落点）**。打开 MT6835 数据手册 SPI 章节，把「读角度」事务的命令字节、字节数、角度位域抄成注释与常量（例结构：`CMD_ANGLE = 0x3F`，32-bit 帧，角度在响应的 bit[24:4]——**以手册为准，禁止照抄本例**）。同步记入 `docs/reviews/T4-brief.md`。据此写 `mt6835_raw_to_urad` 的失败测试向量：raw=0 → 0 µrad；raw=2^20（半圈）→ +3141592 或 −3141592（按环绕定义：>π 取负，即 raw ∈ (2^20, 2^21) 映射负角）；zero 平移；invert 翻符号。
- [ ] **步骤 2：实现纯函数与 SPI 读取**。换算整数数学：`urad = (int64_t)raw21 * 6283185 / 2097152`（u64 中间量），减 zero 后 wrap 到 ±3141592。SPI：`spi_bus_initialize(SPI3_HOST, …MOSI=40/MISO=46/SCLK=39…)` + `spi_bus_add_device`（CS=IO7，模式与最大时钟按手册，保守 ≤10 MHz），`spi_device_polling_transmit` 每毫秒一读（模板照抄 as5600 任务结构：CPU1、vTaskDelayUntil、失败计数、is_ok）。
- [ ] **步骤 3：上板自测**：①静止时 raw21 逐毫秒读取稳定（±2 count 抖动内）；②手转摆杆一整圈，`theta_urad` 从 0 连续变到 ±π 跳变回 0 再回 0（圈数守恒的环绕检验）；③拔掉编码器线 → 3 ms 内…（1 kHz×10 次失败）`is_ok` 变 false。`SELF-TEST DONE` 全 PASS。
- [ ] **步骤 4：提交** `git commit -m "mt6835 摆角编码器：SPI 读取 + ±π 环绕换算 + 1 kHz 采样任务"`

### 任务 5：telemetry 200 Hz STATE 流

**文件：**
- 新建：`components/telemetry/`（`telemetry.c/h`、`telemetry_self_test.c`、`CMakeLists.txt`、`Kconfig`）
- 修改：`components/as5600/include/as5600.h`（补 `int64_t as5600_position_um(void)`，见下）

**接口（产出）：**
```c
esp_err_t telemetry_start(void);       /* 200 Hz 任务（CPU1, prio 2, vTaskDelayUntil 5 ms） */
/* 纯函数（自测目标）：组帧，不含 '\n'，返回字节数 */
int telemetry_format_state(char *buf, size_t len, int32_t x_um, int32_t theta_urad,
                           int32_t vx_mm_s, int32_t vth_mrad_s);
```

**换算（契约级，整数精确）：** `x_um = as5600_position_counts() * 625 / 64`（40 mm/4096 count = 625/64 µm/count，u64 中间量）——新增 `as5600_position_um()` 直接以 counts 算，不经 mm 浮点。`vx_mm_s = lround(as5600_velocity_mm_s())`（16 ms 窗口沿用）；`theta_urad/vth_mrad_s` 来自 mt6835（任务 7 接入零位标定后生效）。

- [ ] **步骤 1：失败测试**。`telemetry_format_state` 断言：格式 `"STATE %ld %ld %ld %ld"`、正负号字段、零值；buf 不足 40 字节时返回负值。换算断言：counts=64 → 625 µm；counts=−64 → −625。
- [ ] **步骤 2：实现**。任务循环里快照四个原子量 → 组帧 → `cartproto_send_raw(buf, n)`（含 `\n`）。所有模式无条件发送（DISABLED/FAULT 也发——数据面常开是契约）。
- [ ] **步骤 3：上板自测**：用 USB 串口（CH343P 接 UART1，460800）抓 10 s：行率 = 200±1 Hz；`x_um` 静止漂移 < 1 mm；手推小车 `x_um` 变化方向与推的方向一致。`SELF-TEST DONE` 全 PASS。
- [ ] **步骤 4：提交** `git commit -m "telemetry：200 Hz STATE 行流与整数精确换算"`

### 任务 6：cartproto 解析与应答

**文件：**
- 修改：`components/cartproto/cartproto.c/.h`、`cartproto_self_test.c`（由旧 protocol 自测改造）

**接口（产出）：**
```c
/* 纯解析（自测目标）：合法行返回句柄并填充 req；非法返回 CART_LINE_INVALID（静默丢弃） */
typedef enum { CART_CMD_VEL, CART_CMD_HOME, CART_CMD_RESET, CART_CMD_QUERY,
               CART_CMD_CONFIG_SET, CART_CMD_CONFIG_DUMP } cart_cmd_t;
typedef struct {
    cart_cmd_t cmd;
    int32_t v_mm_s;            /* VEL：整数 mm/s，|v| > 32767 判非法丢弃（防溢出，正常钳位在 cartfsm） */
    char name[16]; long value; /* CONFIG */
} cart_req_t;
bool cartproto_parse_line(const char *line, cart_req_t *out);
/* 应答原语（cartfsm/telemetry 也会用） */
void cartproto_send_mode(const char *mode, const char *reason);  /* "MODE <mode> <reason>\n" */
void cartproto_send_ok(const char *name, long value);
void cartproto_send_err(const char *name);
/* 分发回调：cartfsm 在任务 7 注册 */
typedef void (*cartproto_exec_cb_t)(const cart_req_t *req);
void cartproto_register_exec(cartproto_exec_cb_t cb);
```

**解析契约（protocol.md 命令节）：** 词法 = 单空格分隔、命令词全大写、行尾仅容空白；`VEL` 整数（容许负号）；`CONFIG NAME value` 的 NAME ∈ `[A-Z0-9_]{1,15}`、value 为十进制 long；`CONFIG?` dump；非法行（含空行、乱码、超长截断行）静默丢弃且不影响后续行。

- [ ] **步骤 1：失败测试**。合法集：六命令各至少 1 例 + `VEL -200` + `CONFIG X_LIMIT 250000`。非法集：小写 `vel 200`、`VEL 1.5`、`VEL abc`、`VEL 200 300`、`CONFIG x 1`、`CONFIG VEL_MAX`（缺值）、`CONFIG? extra`、`""`、`HOME NOW`、带 `\r` 尾。逐条断言。
- [ ] **步骤 2：实现**。分发：收到合法行回调 exec（未注册时仅丢弃——保证任务 1 骨架语义不变）。
- [ ] **步骤 3：上板自测 + 环回**：USB 串口手动发正反用例，观察非法行无任何输出、`QUERY`（exec 尚未实现时可暂由占位回 `MODE DISABLED NONE` 验证管线）。`SELF-TEST DONE` 全 PASS。
- [ ] **步骤 4：提交** `git commit -m "cartproto：命令行解析、非法行静默丢弃、MODE/OK/ERR 应答原语"`

### 任务 7：cartfsm 模式机 + CONFIG 注册表

**文件：**
- 新建：`components/cartfsm/`（`cartfsm.c/h`、`params.c` 并入该组件、`cartfsm_self_test.c`、`CMakeLists.txt`、`Kconfig`）
- 修改：`components/tmc2209/tmc2209.c/.h`（新增 `esp_err_t tmc2209_set_stall_threshold(uint8_t sgthrs)`——写 `REG_SGTHRS`）、`main/app_main.c`（接线：注册 exec、启动 fsm）

**接口（产出）：**
```c
esp_err_t cartfsm_start(void);   /* 1 kHz 监督任务（CPU1, prio 4），上电 DISABLED、EN 关 */
/* 模式与原因（与 protocol.md 枚举一一对应） */
typedef enum { CM_DISABLED, CM_HOMING, CM_READY, CM_FAULT } cm_mode_t;
typedef enum { CR_NONE, CR_STEP_LOSS, CR_STALL, CR_AS5600, CR_MT6835, CR_HOME_TIMEOUT } cm_reason_t;
/* 参数注册表：protocol.md §参数注册表 十二项全量 */
typedef struct {
    long vel_max, acc_max, wdt_ms, wdt_dec, home_vel, home_timeout_ms, x_limit_um;
    long pole_zero_urad, pole_inv, x_inv, sgthrs, step_loss_um;  /* 台位标定项 */
} fw_params_t;
const fw_params_t *fw_params(void);   /* 快照，volatile 存储 */
```

**模式机（契约，转换表完整分类，实现须以表驱动或等价穷举）：**

| 当前态 | 事件 | 次态 / 动作 |
|---|---|---|
| DISABLED | `HOME` | HOMING：EN 开，vel_target(−HOME_VEL)，记超时戳，推 MODE |
| HOMING | 堵转触发（DIAG1 或 SG 轮询） | READY：急停，`as5600_set_origin(-265.0)`，推 MODE |
| HOMING | 超时 `HOME_TIMEOUT` | FAULT/HOME_TIMEOUT：斜坡停后 EN 关，推 MODE |
| HOMING | AS5600 或 MT6835 读取失败 | FAULT/AS5600|MT6835：同上 |
| READY | `VEL` | 换算 steps（×40）钳 ±VEL_MAX 后 `vel_target`（软限位见下） |
| READY | 丢步（\|E−C\| > STEP_LOSS） | FAULT/STEP_LOSS |
| READY | 堵转 | FAULT/STALL |
| READY | 编码器失败 | FAULT/AS5600|MT6835 |
| FAULT | `RESET` | DISABLED（EN 已关），推 MODE |
| 任意 | `QUERY` | 回当前 `MODE <mode> <reason>` |
| 任意 | `CONFIG`/`CONFIG?` | 按 protocol.md 命令×模式矩阵（HOMING/FAULT 拒绝 SET 回 ERR） |

软限位（契约）：监督任务每毫秒读 `x_um`，`|x| > X_LIMIT` 时把与越界同号的目标分量替换为 0（实现：钳 `vel_target` 的符号，向内不受限）。

标定量作用点：`X_INV` 由 cartfsm 写 `step_engine_set_dir_invert()`——引擎位置计数含反转后即物理域，与 AS5600 的 E 同域，E−C 判据无需再换算；`POLE_ZERO_URAD`/`POLE_INV` 在 telemetry 组帧时应用于 `mt6835_theta_urad()` 的原始角（mt6835 组件保持纯传感器，不读参数）。速度换算统一走 cartfsm 内一个辅助函数 `mmps_to_q16(long v_mm_s)`（×40 steps/s 再 ×65536，u64 中间量），VEL 与回零共用。

- [ ] **步骤 1：失败测试（纯逻辑）**。转换表逐行写成用例（含 `HOME` 在 READY/FAULT 被忽略、`RESET` 在非 FAULT 被忽略、`VEL` 在非 READY 被忽略）；CONFIG 矩阵 4 模式 × 2 命令；参数范围表（protocol.md §参数注册表 逐项边界值 ±1）；扩项各自范围。运行确认失败。
- [ ] **步骤 2：实现**。`params.c`：默认值常量表 + `fw_params_set(name, value)`（范围与模式校验）；监督任务：读 as5600_is_ok/mt6835_is_ok、算 `\|E−C\|`（E=counts×1600/4096=counts×125/32 对齐到 steps 域，u64）、软限位钳位、堵转源（任务 8 接入，先留接口 `cartfsm_report_stall(bool)`）；EN 与 MODE 推送在转换点原子完成。
- [ ] **步骤 3：上板自测**：脚本化串口会话（发命令序列、断言应答行序列，可用固件仓库 `tools/host_sim.py` 改造为 protocol 版 `tools/cartpole_host.py`——预估，允许手测替代）。`SELF-TEST DONE` 全 PASS。
- [ ] **步骤 4：提交** `git commit -m "cartfsm：四态模式机、CONFIG 注册表、软限位与命令矩阵"`

### 任务 8：堵转回零与 StallGuard

**文件：**
- 修改：`components/tmc2209/tmc2209.c/.h`（`set_stall_threshold` 若未在任务 7 落地则在此落地）、`components/cartfsm/cartfsm.c`（回零流程 + DIAG1 采样）、`components/bsp/bsp.c`（DIAG1 上升沿捕获：1 kHz 电平轮询即可，GPIO ISR 非必需——预估）

- [ ] **步骤 1：失败测试（流程级，纯逻辑）**。注入假 stall 报告与时钟，断言：HOME → HOMING →（stall）→ READY 且 `as5600_set_origin(-265)` 被调用（用 spy）；HOME → 超时 → FAULT/HOME_TIMEOUT；HOMING 中 AS5600 失败 → FAULT/AS5600 优先于超时。
- [ ] **步骤 2：实现**。EN 开后按 `fw_params()->sgthrs` 写 `tmc2209_set_stall_threshold()`；回零速度 −HOME_VEL 经 `vel_target`；1 kHz 轮询 DIAG1 电平（`gpio_get_level`，CONFIG_GPIO16），高电平视为堵转（极性按 TMC2209 DIAG 配置——**上板用示波器/逻辑分析仪或低速手堵验证极性，未决事项 2**）。
- [ ] **步骤 3：上板验收**（机械台位在位）：小车置行程中段，发 `HOME`：观察向 −x 低速运动、触端堵转、停在端部、MODE READY；连续 5 次回零，`x_um` 起点散布记录进 `docs/reviews/T8-acceptance.md`（protocol.md 预期 ±1–2 mm）。
- [ ] **步骤 4：提交** `git commit -m "回零：StallGuard 堵转检测 + 原点重置 + 超时故障"`

### 任务 9：监督闭环与故障路径

**文件：**
- 修改：`components/cartfsm/cartfsm.c`（丢步判定已有骨架，此处补 FAULT 停机次序）、`cartfsm_self_test.c`

**契约：** FAULT 进入次序 = 目标斜坡置 0（WDT_DEC 档）→ 等 `vel_now == 0` → EN 关 → 推 MODE（斜坡停机后关闭使能，protocol.md §模式机）。FAULT 下 STATE 流继续（θ 恒有效，x 保留最后值）。

- [ ] **步骤 1：失败测试**：①READY 中拔 AS5600 → ≤15 ms 内 MODE FAULT AS5600 且 EN 关（1 kHz×10 次失败窗口 + 余量）；②拔 MT6835 同理；③人为制造 C−E 偏差（测试钩子注入假 counts）→ STEP_LOSS；④RESET 后可重新 HOME 全流程。
- [ ] **步骤 2：实现与上板验收**（拔线类用例在台位可承受时执行，否则以注入钩子为准并在验收记录注明）。
- [ ] **步骤 3：提交** `git commit -m "故障监督闭环：编码器失联/丢步/堵转 → FAULT 斜坡停机 → EN 关"`

### 任务 10：端到端联调与文档定稿

**文件：**
- 新建：固件仓库 `tools/cartpole_host.py`（若任务 7 未建）、`docs/reviews/T10-acceptance.md`
- 修改：固件仓库 `README.md`（重写为 cartpole 固件说明：硬件表 + 引脚表 + 协议指向 protocol.md + 构建/烧录/自测说明）、删除旧 `PLAN.md`（历史在 git）
- 修改：本仓库 `deploy/v2/edge/config.h`（`CFG_SERIAL_PORT` 若在 PC 联调中发现新设备名）

**验收用例（契约逐条对应 protocol.md）：**
1. 符号约定标定（protocol.md §符号约定）：`VEL 200` 实际方向 = +x；手掰摆杆同向 `theta_urad` 增大；不符先改固件符号（X_INV/POLE_INV）。
2. 端侧 CLI 全形态：`query/state/vel 200 --hold 1/config VEL_MAX 800/home/console` 逐条过（用 PC 端侧程序经 CH343P 直连，或在 CV610 上跑）。
3. 看门狗：`vel 200` 后 kill 端侧进程 → 100 ms 内减速到 0 且 `query` 仍回 READY；重启端侧恢复跟踪。
4. 200 Hz 流 + 460800 长稳：10 min 无错行（端侧按 proto 解析统计 UNKNOWN 行数 = 0）。
5. 全流程：HOME → 小速度回中 → 手扶杆 → （有模型时）策略接管；无模型则 `vel` 手动左右打验证换向。
- [ ] **步骤 1–5：逐用例执行并记录**（含失败重跑）；全部通过后重写 README、删除 PLAN.md。
- [ ] **步骤 6：提交** `git commit -m "端到端验收通过，README 定稿"`，并在本仓库提交 edge 侧联调修正（如有）。

## Review Focus（规格隐含但易漏的输入类，均已锚定测试）

1. **看门狗归零后命令恢复继续跟踪**（不是重新 HOME）——任务 3 ISR 自测③ + 任务 10 用例 3。
2. **换向经零速的 DIR 建立时间**（丢 2 µs 会导致首步方向错误）——任务 3 步数守恒断言。
3. **软限位在回程方向放行**（回零后停在 −265 mm 处于限外，向内运动不受阻）——任务 7 步骤 1 补一条：`x=−270 mm, target=−100 mm/s` → 目标变 0；`x=−270 mm, target=+100 mm/s` → 不钳。
4. **STATE 在 DISABLED/FAULT 常开**——任务 5 自测（模式无关发送）+ 任务 9 用例①拔线后流不断。
5. **运行中收紧 CONFIG 立即约束当前运动**（X_LIMIT 调小、ACC_MAX 调低立即生效）——任务 7 步骤 1 补：READY 中 `X_LIMIT 10000` 后位置超限时目标被钳。

## 自审记录

- 规格覆盖：protocol.md §数据面（任务 5）、§模式机（7/8/9）、§命令（6/7）、§参数注册表（7）、§看门狗（3）、§软限位（7）、§符号约定（10）、§操作流程（10）——逐节有落点。
- 未决事项 5 项单独成节，未混入契约。
- 接口签名在任务间交叉引用处（vel_target/vel_now、theta_urad、send_mode、params）已逐一核对一致。
